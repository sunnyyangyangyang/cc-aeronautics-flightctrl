-- ============================================================
-- Aeronautics Fixed-Wing Flight Controller
-- Main Program
--
-- Requires: CC: Tweaked, CC: Sable, Create, Create Aeronautics
--
-- Peripheral setup (see config/settings.lua for details):
--   bottom:  Gimbal Sensor (attitude)
--   top:     Altitude Sensor
--   back:    Velocity Sensor (facing forward)
--   front:   Navigation Table (optional)
--   right:   Sequenced Gearshift -> Elevator
--   left:    Sequenced Gearshift -> Left Aileron
--   bottom:  Sequenced Gearshift -> Right Aileron
--   top:     Sequenced Gearshift -> Rudder
--   back:    Rotation Speed Controller -> Propeller
-- ============================================================

-- Load modules
local PID = require("lib/pid")
local Hardware = require("lib/hardware")
local Controller = require("lib/controller")
local CONFIG = require("config/settings")

-- ============================================================
-- Flight Controller
-- ============================================================
local FC = {}

function FC:init()
    -- Initialize hardware
    self.hw = Hardware.new(CONFIG)
    self.hw:initPeripherals()

    -- Initialize flight controller (abstracted PID cascade)
    self.ctrl = Controller.new(CONFIG)

    -- Flight state
    self.mode = "IDLE"          -- IDLE, MANUAL, AUTO, HOVER, CRUISE, LANDING, EMERGENCY
    self.running = true

    -- Logging
    self.log_enabled = false
    self.log_file = nil
    self.log_tick = 0

    -- Targets
    self.targets = {
        airspeed = CONFIG.defaults.target_airspeed,
        altitude = CONFIG.defaults.target_altitude,
        roll = 0,
        heading = 0,
        pitch = 0,              -- Inner loop pitch target (set by altitude PID)
    }

    -- Timing
    self.last_display_update = 0

    -- Manual control state
    self.manual = {
        throttle = CONFIG.limits.idle_throttle_rpm,
        elevator = 0,
        ailerons = 0,
        rudder = 0,
    }

    print("")
    print("------------------------------------------------")
    print(string.format("|   Aeronautics Flight Controller v%-4s |", CONFIG.version))
    print("|   Type 'help' for commands                   |")
    print("------------------------------------------------")
    print("")
    print("[FC] System initialized. Sensors: gimbal, altitude, velocity")
    print("[FC] Awaiting pilot commands...")
    print("")
end

-- ============================================================
-- Sensor reading
-- ============================================================
function FC:readSensors()
    return self.hw:readSensors()
end

-- ============================================================
-- Display update
-- ============================================================
function FC:updateDisplay()
    local now = os.clock()
    if now - self.last_display_update < CONFIG.loop.display_rate then
        return
    end
    self.last_display_update = now

    local sensors = self.hw.sensors
    self.hw:updateDisplay(sensors, self.mode, self.targets)
end

-- ============================================================
-- Flight modes
-- ============================================================

--- Manual mode: keyboard-controlled flight
function FC:modeManual()
    self.mode = "MANUAL"
    self:resetPIDs()
    print("[FC] MANUAL mode - Arrow keys: pitch/roll, Q/A: throttle, E: center, S: stop")

    -- Reset manual state
    self.manual.throttle = CONFIG.limits.idle_throttle_rpm
    self.manual.elevator = 0
    self.manual.aileron = 0
    self.manual.rudder = 0

    while self.mode == "MANUAL" and self.running do
        local event, p1, p2, p3 = os.pullEvent()

        if event == "terminate" then
            self:emergencyStop()
            break
        elseif event == "key" then
            -- Pitch control (up/down arrows)
            if p1 == keys.up then
                self.manual.elevator = math.min(CONFIG.limits.max_elevator_angle,
                    self.manual.elevator + 3)
            elseif p1 == keys.down then
                self.manual.elevator = math.max(-CONFIG.limits.max_elevator_angle,
                    self.manual.elevator - 3)
            -- Roll control (left/right arrows)
            elseif p1 == keys.left then
                self.manual.aileron = math.max(-CONFIG.limits.max_aileron_angle,
                    self.manual.aileron - 3)
            elseif p1 == keys.right then
                self.manual.aileron = math.min(CONFIG.limits.max_aileron_angle,
                    self.manual.aileron + 3)
            -- Throttle
            elseif p1 == keys.q then
                self.manual.throttle = math.min(CONFIG.limits.max_throttle_rpm,
                    self.manual.throttle + 16)
            elseif p1 == keys.a then
                self.manual.throttle = math.max(0,
                    self.manual.throttle - 16)
            -- Center controls
            elseif p1 == keys.e then
                self.manual.elevator = 0
                self.manual.aileron = 0
                self.manual.rudder = 0
            -- Stop engine
            elseif p1 == keys.s then
                self.manual.throttle = 0
            -- Yaw (W/D)
            elseif p1 == keys.w then
                self.manual.rudder = math.max(-CONFIG.limits.max_rudder_angle,
                    self.manual.rudder - 2)
            elseif p1 == keys.d then
                self.manual.rudder = math.min(CONFIG.limits.max_rudder_angle,
                    self.manual.rudder + 2)
            end

            -- Apply controls
            self.hw:setThrottle(self.manual.throttle)
            self.hw:setElevator(self.manual.elevator)
            self.hw:setAilerons(self.manual.aileron)
            self.hw:setRudder(self.manual.rudder)

            self:readSensors()
            self:updateDisplay()
        end
    end
end

--- Auto mode: full autopilot with altitude + airspeed hold
function FC:modeAuto(targetAirspeed, targetAltitude)
    self.mode = "AUTO"
    self.targets.airspeed = targetAirspeed or CONFIG.defaults.target_airspeed
    self.targets.altitude = targetAltitude or CONFIG.defaults.target_altitude
    self.targets.roll    = 0
    self.targets.heading = 0
    self:resetPIDs()

    local has_sublevel = sublevel and sublevel.isInPlotGrid and sublevel.isInPlotGrid()
    local has_gimbal   = self.hw.peripherals.gimbal and self.hw.peripherals.gimbal.getAngles

    if has_sublevel then
        print("[FC] OK: Using CC:Sable sublevel API for altitude/velocity data.")
    else
        print("[FC] WARNING: Not on Sable Sub-Level! Altitude/velocity from peripherals only.")
    end
    if not has_gimbal then
        print("[FC] WARNING: No Gimbal Sensor! Pitch/roll control will be blind.")
    end

    print("[FC] AUTO mode - Speed: " .. self.targets.airspeed ..
          ", Alt: " .. self.targets.altitude)
    print("[FC] Phases: TAKEOFF -> CLIMB -> CRUISE")

    local tick      = 0
    local phase     = "TAKEOFF"
    local ground_alt = nil

    while self.mode == "AUTO" and self.running do
        tick         = tick + 1
        self.log_tick = self.log_tick + 1

        local s  = self:readSensors()
        local dt = CONFIG.loop.update_rate

        if ground_alt == nil then
            ground_alt = s.altitude
            print(string.format(
                "[AUTO] Ground alt (abs Y): %.2f | Target alt (abs Y): %.2f | Rise needed: %.1f",
                ground_alt,
                self.targets.altitude,
                self.targets.altitude - ground_alt
            ))
            if self.targets.altitude <= ground_alt + 5 then
                print("[AUTO] WARNING: target altitude is at or below ground! " ..
                      "Use 'setalt' to set an absolute Y value above " ..
                      string.format("%.0f", ground_alt + 20))
            end
        end

        local relative_alt = s.altitude - ground_alt

        local tgt = {
            altitude = self.targets.altitude,
            airspeed = self.targets.airspeed,
            roll     = 0,
            heading  = 0,
        }

        -- Phase: TAKEOFF
        if phase == "TAKEOFF" then
            tgt.airspeed = self.targets.airspeed
            tgt.roll     = 0
            tgt.altitude = ground_alt

            local airborne = relative_alt > 3 and s.vertical_speed > 0.3
            local timeout  = tick > 400

            if airborne then
                phase = "CLIMB"
                self:resetPIDs()
                print(string.format("[AUTO] Phase: CLIMB (airborne, rel_alt=%.1f vs=%.2f)",
                    relative_alt, s.vertical_speed))
            elseif timeout then
                phase = "CLIMB"
                self:resetPIDs()
                print(string.format("[AUTO] Phase: CLIMB (timeout, rel_alt=%.1f)",
                    relative_alt))
            end

        -- Phase: CLIMB
        elseif phase == "CLIMB" then
            tgt.airspeed = math.max(128, self.targets.airspeed * 0.8)

            if math.abs(s.altitude - self.targets.altitude) < 5 then
                phase = "CRUISE"
                self:resetPIDs()
                print("[AUTO] Phase: CRUISE")
            end

        -- Phase: CRUISE
        elseif phase == "CRUISE" then
        end

        -- Run controller
        local cmd = self.ctrl:update(tgt, s, dt)

        local elevator_cmd = cmd.elevator
        local aileron_cmd  = cmd.aileron
        local rudder_cmd   = cmd.rudder
        local throttle_cmd = cmd.throttle

        -- Phase overrides
        if phase == "TAKEOFF" then
            throttle_cmd = CONFIG.limits.max_throttle_rpm
            elevator_cmd = math.max(-15, math.min(15, elevator_cmd))

        elseif phase == "CLIMB" then
            if math.abs(s.altitude - self.targets.altitude) > 10 then
                throttle_cmd = CONFIG.limits.max_throttle_rpm
            end
        end

        -- Logging
        if self.log_enabled and self.log_file and self.log_tick % 5 == 0 then
            local log_line = string.format(
                "%d,%s,%+.2f,%+.2f,%.2f,%.2f,%.2f,%.2f,%+.2f,%+.2f,%+.2f,%+.2f,%+.1f,%+.1f,%+.1f,%+.0f",
                tick, phase,
                s.pitch, s.roll,
                s.altitude, cmd.filteredSpeed, s.vertical_speed,
                relative_alt,
                cmd.altError, cmd.pitchError, cmd.rollError, cmd.pitchTarget,
                elevator_cmd, aileron_cmd, rudder_cmd, throttle_cmd)
            self.log_file.write(log_line .. "\n")
            self.log_file.flush()
        end

        -- Apply controls
        self.hw:setThrottle(throttle_cmd)
        self.hw:setElevator(elevator_cmd)
        self.hw:setAilerons(aileron_cmd)
        self.hw:setRudder(rudder_cmd)

        self:updateDisplay()

        -- Console status every 20 ticks
        if tick % 20 == 0 then
            print(string.format(
                "[AUTO] #%-4d %-8s P:%+5.1f R:%+5.1f | AbsAlt:%6.1f RelAlt:%+5.1f VS:%+.2f Spd:%5.1f | Elv:%+5.1f Ail:%+5.1f Thr:%3.0f",
                tick, phase,
                s.pitch, s.roll,
                s.altitude, relative_alt, s.vertical_speed, cmd.filteredSpeed,
                elevator_cmd, aileron_cmd, throttle_cmd))
        end

        os.sleep(dt)
    end
end

--- Hover mode: maintain position (airspeed ~ 0, hold altitude)
function FC:modeHover(targetAltitude)
    self.targets.altitude = targetAltitude or self.hw.sensors.altitude
    self.targets.airspeed = 0
    self:modeAuto(0, self.targets.altitude)
    self.mode = "HOVER"
    print("[FC] HOVER mode - maintaining altitude: " .. self.targets.altitude)

    while self.mode == "HOVER" and self.running do
        os.sleep(CONFIG.loop.update_rate)

        local ev = os.pullEvent(0)
        if ev == "terminate" then
            self:emergencyStop()
            break
        end

        local s = self:readSensors()
        local tgt = { altitude = self.targets.altitude, airspeed = 0, roll = 0, heading = 0 }
        local cmd = self.ctrl:update(tgt, s, CONFIG.loop.update_rate)

        self.hw:setThrottle(cmd.throttle)
        self.hw:setElevator(cmd.elevator)
        self.hw:setAilerons(cmd.aileron)
        self.hw:setRudder(cmd.rudder)

        self:updateDisplay()
    end
end

--- Landing mode: gradual descent to ground
function FC:modeLand()
    self.mode = "LANDING"
    self:resetPIDs()
    print("[FC] LANDING - initiating descent...")

    print("[FC] Phase 1: Reducing airspeed...")
    local current_throttle = self.manual.throttle or CONFIG.limits.idle_throttle_rpm
    for rpm = current_throttle, 32, -4 do
        if not self.running or self.mode ~= "LANDING" then break end
        self.hw:setThrottle(rpm)
        self:readSensors()
        self:updateDisplay()
        os.sleep(0.15)
    end

    print("[FC] Phase 2: Descending...")
    while self.running and self.mode == "LANDING" do
        os.sleep(CONFIG.loop.update_rate)
        local s = self:readSensors()

        if s.altitude < 3 then
            print("[FC] Touchdown imminent!")
            break
        end

        local tgt = { altitude = 0, airspeed = 32, roll = 0, heading = 0 }
        local cmd = self.ctrl:update(tgt, s, CONFIG.loop.update_rate)

        self.hw:setThrottle(cmd.throttle)
        self.hw:setElevator(cmd.elevator)
        self.hw:setAilerons(cmd.aileron)
        self.hw:setRudder(cmd.rudder)

        self:updateDisplay()
    end

    print("[FC] Phase 3: Stopping...")
    self.hw:setThrottle(0)
    self.hw:neutralize()
    self.hw:resetSurfaceTracking()
    self.mode = "LANDED"
    print("[FC] LANDED safely!")
    self:updateDisplay()
end

-- ============================================================
-- Safety functions
-- ============================================================
function FC:emergencyStop()
    self.mode = "EMERGENCY"
    print("[FC] EMERGENCY STOP!")

    self.hw:stopThrottle()
    self.hw:neutralize()
    self.hw:resetSurfaceTracking()

    if CONFIG.safety.auto_level_on_emergency then
        local s = self:readSensors()
        if math.abs(s.roll) > 10 then
            self.ctrl:reset()
            for i = 1, 10 do
                s = self:readSensors()
                local tgt = { altitude = s.altitude, airspeed = 0, roll = 0, heading = 0 }
                local cmd = self.ctrl:update(tgt, s, 0.1)
                self.hw:setAilerons(cmd.aileron)
                os.sleep(0.1)
            end
        end
    end

    self.hw:neutralize()
    self:updateDisplay()
    self.running = false
end

function FC:resetPIDs()
    self.ctrl:reset()
end

-- ============================================================
-- Command handling
-- ============================================================
function FC:handleCommand(input)
    if not input or input == "" then return end

    local parts = {}
    for word in input:gmatch("%S+") do
        table.insert(parts, word)
    end

    local cmd = parts[1]:lower()
    local arg1 = tonumber(parts[2])
    local arg2 = tonumber(parts[3])

    if cmd == "help" then
        self:printHelp()
    elseif cmd == "manual" then
        self:switchMode(function() self:modeManual() end)
    elseif cmd == "auto" then
        self:switchMode(function() self:modeAuto(arg1, arg2) end)
    elseif cmd == "hover" then
        self:switchMode(function() self:modeHover(arg1) end)
    elseif cmd == "land" then
        self:switchMode(function() self:modeLand() end)
    elseif cmd == "stop" then
        self.running = false
        self.hw:stopThrottle()
        self.hw:neutralize()
        print("[FC] System stopped.")
    elseif cmd == "emergency" or cmd == "panic" then
        self:emergencyStop()
    elseif cmd == "status" then
        self:readSensors()
        local s = self.hw.sensors
        print("[FC] Status:")
        print(string.format("  Mode: %s", self.mode))
        print(string.format("  Pitch: %+4.1f°  Roll: %+4.1f°", s.pitch, s.roll))
        print(string.format("  Alt: %.1f  Speed: %.1f", s.altitude, s.airspeed))
        print(string.format("  V/S: %.1f  Pressure: %.2f", s.vertical_speed, s.air_pressure))
        if sublevel and sublevel.isInPlotGrid and sublevel.isInPlotGrid() then
            local av = sublevel.getAngularVelocity()
            print(string.format("  AngVel: P:%.2f R:%.2f Y:%.2f", av.x or 0, av.z or 0, av.y or 0))
        end
    elseif cmd == "setalt" then
        self.targets.altitude = arg1 or self.targets.altitude
        print("[FC] Target altitude: " .. self.targets.altitude)
    elseif cmd == "setspeed" then
        self.targets.airspeed = arg1 or self.targets.airspeed
        print("[FC] Target airspeed: " .. self.targets.airspeed)
    elseif cmd == "throttle" then
        self.hw:setThrottle(arg1 or 0)
        print("[FC] Throttle set to " .. (arg1 or 0) .. " RPM")
    elseif cmd == "test" then
        self:testControls()
    elseif cmd == "log-on" then
        self:enableLog()
    elseif cmd == "log-off" then
        self:disableLog()
    elseif cmd == "log" then
        print(string.format("[FC] Log: %s (type 'log-on' or 'log-off')",
            self.log_enabled and "ENABLED" or "DISABLED"))
    elseif cmd == "reset" or cmd == "center" then
        -- 一键归零：先向执行器发送归零命令，再重置追踪和 PID
        print("[FC] Resetting all control surfaces to center...")
        -- 主动下达归零命令给所有执行器（基于当前追踪的角度计算 delta）
        self.hw:setElevator(0)
        self.hw:setAilerons(0)
        self.hw:setRudder(0)
        -- 等待命令执行完成
        os.sleep(0.2)
        -- 重置追踪状态（确保内部状态与实际一致）
        self.hw:resetSurfaceTracking()
        -- 重置 PID 状态
        self:resetPIDs()
        print("[FC] All control surfaces centered, tracking and PIDs reset.")
    else
        print("[FC] Unknown command: " .. cmd)
        print("[FC] Type 'help' for available commands")
    end
end

function FC:printHelp()
    print("")
    print("  === Flight Controller Commands ===")
    print("  manual              - Manual keyboard control")
    print("  auto [spd] [alt]    - Autopilot (default: 128 RPM, 80m)")
    print("  hover [alt]         - Hover at altitude")
    print("  land                - Auto landing sequence")
    print("  throttle <rpm>      - Set throttle directly")
    print("  setalt <y>          - Set target altitude")
    print("  setspeed <rpm>      - Set target airspeed")
    print("  status              - Show sensor readings")
    print("  test                - Test all control surfaces")
    print("  reset / center      - Reset all surfaces to zero + clear tracking")
    print("  log-on / log-off    - Enable/disable flight data logging")
    print("  emergency / panic   - Emergency stop")
    print("  stop                - Shutdown system")
    print("  help                - Show this help")
    print("")
end

function FC:switchMode(modeFunc)
    -- Save current throttle for smooth transition
    if self.manual then
        self.manual.throttle = self.hw.peripherals.throttle and
            self.hw.peripherals.throttle.getTargetSpeed() or 64
    end

    -- Run mode in parallel so we can still receive commands
    local run, cancel = parallel.waitForAny(
        modeFunc,
        function()
            -- Command listener
            while self.running do
                write("[FC] > ")
                local input = read()
                if input then
                    self:handleCommand(input)
                end
            end
        end
    )
end

function FC:testControls()
    print("[FC] === Control Surface Test ===")

    -- Strict check: report which actuators are available
    local actuator_configs = {
        {name = "elevator",     side = self.hw.config.peripherals.elevator},
        {name = "aileron_left", side = self.hw.config.peripherals.aileron_left},
        {name = "aileron_right",side = self.hw.config.peripherals.aileron_right},
        {name = "rudder",       side = self.hw.config.peripherals.rudder},
        {name = "throttle",     side = self.hw.config.peripherals.throttle},
    }

    local found = {}
    local missing = {}

    for _, act in ipairs(actuator_configs) do
        if not act.side then
            table.insert(missing, act.name .. "(not configured)")
        elseif peripheral.isPresent(act.side) then
            local ptype = peripheral.getType(act.side) or "unknown"
            print("[HW] OK   " .. act.name .. " -> " .. act.side .. " (" .. ptype .. ")")
            table.insert(found, act.name)
        else
            table.insert(missing, act.name .. "(" .. act.side .. ")")
        end
    end

    if #missing > 0 then
        print("[HW] WARNING - Missing actuators: " .. table.concat(missing, ", "))
        print("[HW] Tip: Edit config/settings.lua to match your physical setup")
    end

    if #found == 0 then
        print("[FC] ERROR: No actuators found! Cannot run test.")
        print("[FC] Check config/settings.lua peripheral sides match your computer.")
        return
    end

    print("[FC] Found " .. #found .. " actuator(s): " .. table.concat(found, ", "))
    print("[FC] Testing available control surfaces...")

    -- Reset tracking assuming surfaces start at neutral
    self.hw:resetSurfaceTracking()

    -- Test throttle
    if self.hw.peripherals.throttle then
        print("[FC] Throttle: 0 -> 128 -> 0")
        self.hw:setThrottle(0)
        os.sleep(0.5)
        self.hw:setThrottle(128)
        os.sleep(0.5)
        self.hw:setThrottle(0)
        os.sleep(0.5)
    else
        print("[FC] Throttle: SKIPPED (not connected)")
    end

    -- Test elevator
    if self.hw.peripherals.elevator then
        print("[FC] Elevator: up -> down -> center")
        self.hw:setElevator(15)
        os.sleep(0.5)
        self.hw:setElevator(-15)
        os.sleep(0.5)
        self.hw:setElevator(0)
        os.sleep(0.5)
    else
        print("[FC] Elevator: SKIPPED (not connected)")
    end

    -- Test ailerons (works with single-side)
    if self.hw.peripherals.aileron_left or self.hw.peripherals.aileron_right then
        local sides = {}
        if self.hw.peripherals.aileron_left then table.insert(sides, "L") end
        if self.hw.peripherals.aileron_right then table.insert(sides, "R") end
        print("[FC] Ailerons (" .. table.concat(sides, "+") .. "): left -> right -> center")
        self.hw:setAilerons(15)
        os.sleep(0.5)
        self.hw:setAilerons(-15)
        os.sleep(0.5)
        self.hw:setAilerons(0)
        os.sleep(0.5)
    else
        print("[FC] Ailerons: SKIPPED (neither connected)")
    end

    -- Test rudder
    if self.hw.peripherals.rudder then
        print("[FC] Rudder: left -> right -> center")
        self.hw:setRudder(10)
        os.sleep(0.5)
        self.hw:setRudder(-10)
        os.sleep(0.5)
        self.hw:setRudder(0)
        os.sleep(0.5)
    else
        print("[FC] Rudder: SKIPPED (not connected)")
    end

    print("[FC] === Test complete! ===")
end

-- ============================================================
-- Main loop
-- ============================================================
function FC:run()
    self:init()

    -- Main command loop
    while self.running do
        write("[FC] > ")
        local input = read()
        if input then
            self:handleCommand(input)
        end
    end

    -- Cleanup
    self.hw:stopThrottle()
    self.hw:neutralize()
    print("[FC] Goodbye, pilot!")
end

-- ============================================================
-- Logging
-- ============================================================
function FC:enableLog()
    if self.log_enabled then
        print("[FC] Log already enabled.")
        return
    end
    -- Create log file with timestamp
    local timestamp = os.date("%Y%m%d_%H%M%S")
    local filename = "/logs/flight_" .. timestamp .. ".csv"
    -- Ensure logs directory exists
    fs.makeDir("/logs")
    self.log_file = fs.open(filename, "w")
    if not self.log_file then
        print("[FC] ERROR: Failed to create log file at " .. filename)
        return
    end
    -- Write CSV header
    self.log_file.writeLine("tick,phase,pitch,roll,altitude,airspeed,vertical_speed,relative_alt,alt_error,pitch_error,roll_error,pitch_target,elevator,aileron,rudder,throttle")
    self.log_file.flush()
    self.log_enabled = true
    self.log_tick = 0
    print("[FC] Log ENABLED -> " .. filename)
end

function FC:disableLog()
    if not self.log_enabled then
        print("[FC] Log already disabled.")
        return
    end
    if self.log_file then
        self.log_file.close()
        self.log_file = nil
    end
    self.log_enabled = false
    print("[FC] Log DISABLED.")
end

-- Start
local fc = setmetatable({}, {__index = FC})
fc:run()
