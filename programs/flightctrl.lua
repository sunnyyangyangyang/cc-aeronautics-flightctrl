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
local CONFIG = require("config/settings")

-- ============================================================
-- Flight Controller
-- ============================================================
local FC = {}

function FC:init()
    -- Initialize hardware
    self.hw = Hardware.new(CONFIG)
    self.hw:initPeripherals()

    -- Initialize PID controllers from config
    local tp = CONFIG.pid.throttle
    self.pid_throttle = PID.new(tp.kp, tp.ki, tp.kd, tp.integral_max, tp.output_min, tp.output_max)

    local pp = CONFIG.pid.pitch
    self.pid_pitch = PID.new(pp.kp, pp.ki, pp.kd, pp.integral_max, pp.output_min, pp.output_max)

    local rp = CONFIG.pid.roll
    self.pid_roll = PID.new(rp.kp, rp.ki, rp.kd, rp.integral_max, rp.output_min, rp.output_max)

    local yp = CONFIG.pid.yaw
    self.pid_yaw = PID.new(yp.kp, yp.ki, yp.kd, yp.integral_max, yp.output_min, yp.output_max)

    local ap = CONFIG.pid.altitude
    self.pid_altitude = PID.new(ap.kp, ap.ki, ap.kd, ap.integral_max, ap.output_min, ap.output_max)

    -- Flight state
    self.mode = "IDLE"          -- IDLE, MANUAL, AUTO, HOVER, CRUISE, LANDING, EMERGENCY
    self.running = true

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
    print("╔══════════════════════════════════════════════╗")
    print("║   Aeronautics Flight Controller v1.0        ║")
    print("║   Type 'help' for commands                   ║")
    print("╚══════════════════════════════════════════════╝")
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
    self.targets.roll = 0
    self:resetPIDs()

    print("[FC] AUTO mode - Speed: " .. self.targets.airspeed ..
          ", Alt: " .. self.targets.altitude)

    while self.mode == "AUTO" and self.running do
        os.sleep(CONFIG.loop.update_rate)

        -- Check for termination
        local ev = os.pullEvent(0)
        if ev == "terminate" then
            self:emergencyStop()
            break
        end

        -- Read sensors
        local s = self:readSensors()

        -- === Altitude PID (outer loop) ===
        local alt_error = self.targets.altitude - s.altitude
        local pitch_correction = self.pid_altitude:update(alt_error, CONFIG.loop.update_rate)

        -- === Pitch PID (inner loop) ===
        local pitch_target = pitch_correction  -- Target pitch from altitude PID
        local pitch_error = pitch_target - s.pitch
        local elevator_cmd = self.pid_pitch:update(pitch_error, CONFIG.loop.update_rate)

        -- === Roll PID ===
        local roll_error = self.targets.roll - s.roll
        local aileron_cmd = self.pid_roll:update(roll_error, CONFIG.loop.update_rate)

        -- === Yaw PID (coordinated turn) ===
        -- Use roll rate to estimate turn, apply rudder for coordination
        local yaw_error = -(s.roll_rate or 0) * 0.5  -- Counter-yaw for coordination
        local rudder_cmd = self.pid_yaw:update(yaw_error, CONFIG.loop.update_rate)

        -- === Throttle PID ===
        local speed_error = self.targets.airspeed - s.airspeed
        local throttle_cmd = self.pid_throttle:update(speed_error, CONFIG.loop.update_rate)

        -- === Stall protection ===
        if s.airspeed < CONFIG.safety.stall_speed and s.airspeed > 0 then
            -- Pitch down to gain speed, increase throttle
            elevator_cmd = math.min(elevator_cmd, -10)
            throttle_cmd = math.max(throttle_cmd, 192)
        end

        -- === Ground proximity protection ===
        if s.altitude < CONFIG.safety.ground_proximity_alt and s.altitude > 0 then
            if s.vertical_speed < -2 then
                -- Descending too fast near ground: pitch up
                elevator_cmd = math.max(elevator_cmd, 15)
            end
        end

        -- Apply controls
        self.hw:setThrottle(throttle_cmd)
        self.hw:setElevator(elevator_cmd)
        self.hw:setAilerons(aileron_cmd)
        self.hw:setRudder(rudder_cmd)

        self:updateDisplay()
    end
end

--- Hover mode: maintain position (airspeed ~ 0, hold altitude)
function FC:modeHover(targetAltitude)
    self.targets.altitude = targetAltitude or self.hw.sensors.altitude
    self.targets.airspeed = 0
    self:modeAuto(0, self.targets.altitude)
    -- Override mode name
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

        -- Minimal throttle to maintain altitude
        local alt_error = self.targets.altitude - s.altitude
        local pitch_correction = self.pid_altitude:update(alt_error, CONFIG.loop.update_rate)

        local pitch_error = pitch_correction - s.pitch
        local elevator_cmd = self.pid_pitch:update(pitch_error, CONFIG.loop.update_rate)

        -- Level wings
        local roll_error = 0 - s.roll
        local aileron_cmd = self.pid_roll:update(roll_error, CONFIG.loop.update_rate)

        -- Minimal thrust
        local throttle_cmd = CONFIG.limits.idle_throttle_rpm + alt_error * 2
        throttle_cmd = math.max(CONFIG.limits.min_throttle_rpm,
            math.min(CONFIG.limits.max_throttle_rpm, throttle_cmd))

        self.hw:setThrottle(throttle_cmd)
        self.hw:setElevator(elevator_cmd)
        self.hw:setAilerons(aileron_cmd)
        self.hw:setRudder(0)

        self:updateDisplay()
    end
end

--- Landing mode: gradual descent to ground
function FC:modeLand()
    self.mode = "LANDING"
    self:resetPIDs()
    print("[FC] LANDING - initiating descent...")

    -- Phase 1: Reduce speed
    print("[FC] Phase 1: Reducing airspeed...")
    local current_throttle = self.manual.throttle or CONFIG.limits.idle_throttle_rpm
    for rpm = current_throttle, 32, -4 do
        if not self.running or self.mode ~= "LANDING" then break end
        self.hw:setThrottle(rpm)
        self:readSensors()
        self:updateDisplay()
        os.sleep(0.15)
    end

    -- Phase 2: Gentle descent
    print("[FC] Phase 2: Descending...")
    while self.running and self.mode == "LANDING" do
        os.sleep(CONFIG.loop.update_rate)
        local s = self:readSensors()

        -- Near ground?
        if s.altitude < 3 then
            print("[FC] Touchdown imminent!")
            break
        end

        -- Gentle nose-down for descent
        local descent_pitch = math.max(-15, -(s.altitude / 5))
        local pitch_error = descent_pitch - s.pitch
        local elevator_cmd = self.pid_pitch:update(pitch_error, CONFIG.loop.update_rate)

        -- Keep wings level
        local roll_error = 0 - s.roll
        local aileron_cmd = self.pid_roll:update(roll_error, CONFIG.loop.update_rate)

        -- Maintain minimum safe speed
        local throttle_cmd = math.max(32, s.airspeed - 2)

        self.hw:setThrottle(throttle_cmd)
        self.hw:setElevator(elevator_cmd)
        self.hw:setAilerons(aileron_cmd)
        self.hw:setRudder(0)

        self:updateDisplay()
    end

    -- Phase 3: Stop
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
        -- Try to level wings before stopping
        local s = self:readSensors()
        if math.abs(s.roll) > 10 then
            -- Quick level
            self.pid_roll:reset()
            for i = 1, 10 do
                s = self:readSensors()
                local roll_error = 0 - s.roll
                local aileron_cmd = self.pid_roll:update(roll_error, 0.1)
                self.hw:setAilerons(aileron_cmd)
                os.sleep(0.1)
            end
        end
    end

    self.hw:neutralize()
    self:updateDisplay()
    self.running = false
end

function FC:resetPIDs()
    self.pid_throttle:reset()
    self.pid_pitch:reset()
    self.pid_roll:reset()
    self.pid_yaw:reset()
    self.pid_altitude:reset()
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
                local input = read("[FC] > ")
                if input then
                    self:handleCommand(input)
                end
            end
        end
    )
end

function FC:testControls()
    print("[FC] Testing control surfaces...")
    -- Reset tracking assuming surfaces start at neutral
    self.hw:resetSurfaceTracking()

    -- Test throttle
    print("[FC] Throttle: 0 -> max -> 0")
    self.hw:setThrottle(0)
    os.sleep(0.5)
    self.hw:setThrottle(128)
    os.sleep(0.5)
    self.hw:setThrottle(0)
    os.sleep(0.5)

    -- Test elevator
    print("[FC] Elevator: full up -> full down -> center")
    self.hw:setElevator(15)
    os.sleep(0.5)
    self.hw:setElevator(-15)
    os.sleep(0.5)
    self.hw:setElevator(0)
    os.sleep(0.5)

    -- Test ailerons
    print("[FC] Ailerons: left -> right -> center")
    self.hw:setAilerons(15)
    os.sleep(0.5)
    self.hw:setAilerons(-15)
    os.sleep(0.5)
    self.hw:setAilerons(0)
    os.sleep(0.5)

    -- Test rudder
    print("[FC] Rudder: left -> right -> center")
    self.hw:setRudder(10)
    os.sleep(0.5)
    self.hw:setRudder(-10)
    os.sleep(0.5)
    self.hw:setRudder(0)
    os.sleep(0.5)

    print("[FC] Test complete!")
end

-- ============================================================
-- Main loop
-- ============================================================
function FC:run()
    self:init()

    -- Main command loop
    while self.running do
        local input = read("[FC] > ")
        if input then
            self:handleCommand(input)
        end
    end

    -- Cleanup
    self.hw:stopThrottle()
    self.hw:neutralize()
    print("[FC] Goodbye, pilot!")
end

-- Start
local fc = setmetatable({}, {__index = FC})
fc:run()
