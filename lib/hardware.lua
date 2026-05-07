-- lib/hardware.lua  v1.3.2-patch
-- PATCH list:
--   [1] rotateSmooth(): new unified gearshift driver
--       - separates speed modifier from direction
--       - enforces modifier != 0 (was causing silent no-ops)
--       - caps single-frame delta to max_deg_per_tick to prevent jumps
--   [2] setRudder: was using wrong modifier sign calculation
--   [3] setAilerons: same fix, single-side operation preserved
--   [4] All surface setters use rotateSmooth()

local Hardware = {}
Hardware.__index = Hardware

function Hardware.new(config)
    local self = setmetatable({}, Hardware)
    self.config = config or {}
    self.peripherals = {}
    self.has_aero = false
    self.sensors = {
        pitch = 0, roll = 0, altitude = 0, airspeed = 0,
        air_pressure = 1.0, heading = 0, vertical_speed = 0,
        pitch_rate = 0, roll_rate = 0, yaw_rate = 0,
    }
    self.prev_altitude = 0
    self.prev_time = 0

    self.surface_angles = {
        elevator = 0, aileron_left = 0, aileron_right = 0, rudder = 0,
    }

    -- PATCH [1]: max degrees moved per single rotate() call
    -- Gearshift will still move at its own speed, but we never send a delta
    -- larger than this in one tick — prevents the "catch-up lurch"
    self.max_deg_per_tick = 8   -- tune: lower = smoother but slower tracking

    return self
end

-- ============================================================
-- PATCH [1]: unified smooth gearshift driver
-- @param peripheral  the wrapped Sequenced Gearshift peripheral
-- @param target      desired absolute angle (degrees)
-- @param tracked     current tracked angle (degrees)
-- @param max_angle   hardware clamp (degrees)
-- @param speed_mod   config gearshift_speed_mod (1 or 2 typically)
-- @returns           new tracked angle after this tick's move
-- ============================================================
function Hardware:rotateSmooth(peripheral, target, tracked, max_angle, speed_mod)
    if not peripheral or not peripheral.rotate then
        return tracked
    end

    -- Clamp target to hardware limits
    target = math.max(-max_angle, math.min(max_angle, target))

    local delta = target - tracked
    if math.abs(delta) < 0.5 then
        return tracked   -- close enough, skip (avoid tiny jitter calls)
    end

    -- PATCH: cap the move per tick so we never send a huge single rotate()
    local capped_delta = delta
    if math.abs(capped_delta) > self.max_deg_per_tick then
        capped_delta = math.sign_hw(delta) * self.max_deg_per_tick
    end

    -- rotate(angle, modifier):
    --   angle    = positive integer, how many degrees to rotate
    --   modifier = integer in [-2..2], sign = direction, magnitude = speed
    -- PATCH: keep direction (sign) and speed (magnitude) separate
    local rot_angle = math.max(1, math.floor(math.abs(capped_delta) + 0.5))
    local direction = math.sign_hw(capped_delta)   -- +1 or -1, never 0
    local speed     = math.max(1, math.min(2, math.floor(math.abs(speed_mod or 1))))
    local modifier  = direction * speed            -- e.g. -2, -1, +1, +2

    peripheral.rotate(rot_angle, modifier)

    -- Return the new tracked position (capped, not the full target)
    return tracked + capped_delta
end

-- Simple sign helper (avoids dependency on math.sign_or from controller)
function math.sign_hw(x)
    if x >= 0 then return 1 else return -1 end
end

-- ============================================================
-- initPeripherals (unchanged from original)
-- ============================================================
function Hardware:initPeripherals()
    local sides = self.config.peripherals or {}

    if sides.gimbal     then self.peripherals.gimbal      = peripheral.wrap(sides.gimbal)      end
    if sides.altitude   then self.peripherals.altitude    = peripheral.wrap(sides.altitude)    end
    if sides.velocity   then self.peripherals.velocity    = peripheral.wrap(sides.velocity)    end
    if sides.navigation then self.peripherals.navigation  = peripheral.wrap(sides.navigation)  end
    if sides.elevator   then self.peripherals.elevator    = peripheral.wrap(sides.elevator)    end
    if sides.aileron_left  then self.peripherals.aileron_left  = peripheral.wrap(sides.aileron_left)  end
    if sides.aileron_right then self.peripherals.aileron_right = peripheral.wrap(sides.aileron_right) end
    if sides.rudder     then self.peripherals.rudder      = peripheral.wrap(sides.rudder)      end
    if sides.throttle   then self.peripherals.throttle    = peripheral.wrap(sides.throttle)    end
    if sides.display    then self.peripherals.display     = peripheral.wrap(sides.display)     end

    if aero and aero.getAirPressure then
        self.has_aero = true
        print("[HW] OK   aero API available")
    end
    if sublevel and sublevel.isInPlotGrid then
        print("[HW] OK   sublevel API available")
    end

    local sensor_keys   = {"gimbal","altitude","velocity","navigation"}
    local actuator_keys = {"elevator","aileron_left","aileron_right","rudder","throttle","display"}
    local sensor_count, actuator_count = 0, 0
    local actuator_names, missing_list = {}, {}

    for _, name in ipairs(sensor_keys) do
        local side = (self.config.peripherals or {})[name]
        if not side then
            table.insert(missing_list, name .. "(not configured)")
        elseif peripheral.isPresent(side) then
            print("[HW] OK   " .. name .. " -> " .. side .. " (" .. (peripheral.getType(side) or "?") .. ")")
            sensor_count = sensor_count + 1
        else
            table.insert(missing_list, name .. "(" .. side .. ")")
        end
    end

    for _, name in ipairs(actuator_keys) do
        local side = (self.config.peripherals or {})[name]
        if not side then
            if name == "elevator" or name == "throttle" then
                table.insert(missing_list, name .. "(not configured)")
            end
        elseif peripheral.isPresent(side) then
            print("[HW] OK   " .. name .. " -> " .. side .. " (" .. (peripheral.getType(side) or "?") .. ")")
            actuator_count = actuator_count + 1
            table.insert(actuator_names, name)
        else
            table.insert(missing_list, name .. "(" .. side .. ")")
        end
    end

    print("[HW] Summary: " .. sensor_count .. " sensors, " .. actuator_count ..
          " actuators (" .. table.concat(actuator_names, ", ") .. ")")
    if #missing_list > 0 then
        print("[HW] WARNING - Missing: " .. table.concat(missing_list, ", "))
    end
end

-- ============================================================
-- readSensors (unchanged from original)
-- ============================================================
function Hardware:readSensors()
    local now = os.clock()
    local dt = now - self.prev_time
    if dt <= 0 then dt = 0.05 end
    self.prev_time = now

    if sublevel and sublevel.isInPlotGrid and sublevel.isInPlotGrid() then
        if sublevel.getLogicalPose then
            local ok, pose = pcall(sublevel.getLogicalPose)
            if ok and pose and pose.position and pose.position.y then
                local alt = pose.position.y
                self.sensors.vertical_speed = (alt - self.prev_altitude) / dt
                self.prev_altitude = alt
                self.sensors.altitude = alt
            end
        end
        if sublevel.getVelocity then
            local ok, vel = pcall(sublevel.getVelocity)
            if ok and vel then
                local speed = math.sqrt((vel.x or 0)^2 + (vel.y or 0)^2 + (vel.z or 0)^2)
                self.sensors.airspeed = speed
            end
        end
        if self.sensors.airspeed == 0 and sublevel.getLinearVelocity then
            local ok, linVel = pcall(sublevel.getLinearVelocity)
            if ok and linVel then
                local speed = math.sqrt((linVel.x or 0)^2 + (linVel.y or 0)^2 + (linVel.z or 0)^2)
                self.sensors.airspeed = speed
            end
        end
        if sublevel.getAngularVelocity then
            local ok, angVel = pcall(sublevel.getAngularVelocity)
            if ok and angVel then
                self.sensors.pitch_rate = angVel.x or 0
                self.sensors.roll_rate  = angVel.z or 0
                self.sensors.yaw_rate   = angVel.y or 0
            end
        end
    end

    if self.peripherals.gimbal and self.peripherals.gimbal.getAngles then
        local angles = self.peripherals.gimbal.getAngles()
        self.sensors.pitch = angles[1] or 0
        self.sensors.roll  = angles[2] or 0
    end

    if self.sensors.altitude == 0 and self.peripherals.altitude and self.peripherals.altitude.getHeight then
        local alt = self.peripherals.altitude.getHeight()
        self.sensors.vertical_speed = (alt - self.prev_altitude) / dt
        self.prev_altitude = alt
        self.sensors.altitude = alt
    end

    if self.has_aero and aero.getAirPressure then
        local ok, pressure = pcall(function()
            local pos = vector.new(0, 0, 0)
            if sublevel and sublevel.getLogicalPose then
                local o, p = pcall(sublevel.getLogicalPose)
                if o and p and p.position then pos = p.position end
            end
            return aero.getAirPressure(pos)
        end)
        if ok and pressure then self.sensors.air_pressure = pressure end
    end

    if self.peripherals.altitude and self.peripherals.altitude.getAirPressure then
        local ok, pressure = pcall(self.peripherals.altitude.getAirPressure)
        if ok and pressure then self.sensors.air_pressure = pressure end
    end

    if self.sensors.airspeed == 0 and self.peripherals.velocity and self.peripherals.velocity.getVelocity then
        local ok, speed = pcall(self.peripherals.velocity.getVelocity)
        if ok and speed then self.sensors.airspeed = speed end
    end

    if self.peripherals.navigation and self.peripherals.navigation.getRelativeAngle then
        self.sensors.heading = self.peripherals.navigation.getRelativeAngle() or 0
    end

    return self.sensors
end

-- ============================================================
-- Throttle (Rotation Speed Controller — absolute, no delta needed)
-- ============================================================
function Hardware:setThrottle(rpm)
    if not self.peripherals.throttle then return end
    local max_rpm = self.config.limits.max_throttle_rpm or 256
    local min_rpm = self.config.limits.min_throttle_rpm or 0
    rpm = math.max(min_rpm, math.min(max_rpm, rpm))
    if self.peripherals.throttle.setTargetSpeed then
        self.peripherals.throttle.setTargetSpeed(math.floor(rpm))
    end
end

-- ============================================================
-- PATCH [2]: Elevator — uses rotateSmooth
-- ============================================================
function Hardware:setElevator(angle)
    local max_angle = self.config.limits.max_elevator_angle or 30
    local speed_mod = self.config.limits.gearshift_speed_mod or 1
    self.surface_angles.elevator = self:rotateSmooth(
        self.peripherals.elevator,
        angle,
        self.surface_angles.elevator,
        max_angle,
        speed_mod
    )
end

-- ============================================================
-- PATCH [3]: Ailerons — uses rotateSmooth, single-side safe
-- Differential: left = -angle, right = +angle
-- ============================================================
function Hardware:setAilerons(angle)
    if not self.peripherals.aileron_left and not self.peripherals.aileron_right then return end
    local max_angle = self.config.limits.max_aileron_angle or 25
    local speed_mod = self.config.limits.gearshift_speed_mod or 1

    if self.peripherals.aileron_left then
        self.surface_angles.aileron_left = self:rotateSmooth(
            self.peripherals.aileron_left,
            -angle,   -- differential: opposite direction
            self.surface_angles.aileron_left,
            max_angle,
            speed_mod
        )
    end

    if self.peripherals.aileron_right then
        self.surface_angles.aileron_right = self:rotateSmooth(
            self.peripherals.aileron_right,
            angle,
            self.surface_angles.aileron_right,
            max_angle,
            speed_mod
        )
    end
end

-- ============================================================
-- PATCH [4]: Rudder — uses rotateSmooth
-- Original bug: modifier was computed as modifier * math.abs(speed_mod)
-- which collapsed sign info when speed_mod was fractional
-- ============================================================
function Hardware:setRudder(angle)
    local max_angle = self.config.limits.max_rudder_angle or 20
    local speed_mod = self.config.limits.gearshift_speed_mod or 1
    self.surface_angles.rudder = self:rotateSmooth(
        self.peripherals.rudder,
        angle,
        self.surface_angles.rudder,
        max_angle,
        speed_mod
    )
end

-- ============================================================
-- Utility
-- ============================================================
function Hardware:neutralize()
    self:setElevator(0)
    self:setAilerons(0)
    self:setRudder(0)
end

function Hardware:resetSurfaceTracking()
    self.surface_angles.elevator    = 0
    self.surface_angles.aileron_left  = 0
    self.surface_angles.aileron_right = 0
    self.surface_angles.rudder      = 0
end

function Hardware:stopThrottle()
    self:setThrottle(0)
end

-- ============================================================
-- Display (unchanged)
-- ============================================================
function Hardware:updateDisplay(sensors, mode, targets)
    if not self.peripherals.display then return end
    local disp = self.peripherals.display
    disp.clear()
    disp.setCursorPos(1, 1)
    disp.write(string.format("MODE: %-8s HDG: %4.1f", mode, sensors.heading or 0))
    disp.setCursorPos(1, 2)
    disp.write(string.format("SPD:  %4.1f  %-16s", sensors.airspeed or 0,
        string.rep("#", math.floor((sensors.airspeed or 0) / 10))))
    disp.setCursorPos(1, 3)
    disp.write(string.format("ALT:  %4.1f  %-16s", sensors.altitude or 0,
        string.rep("|", math.floor((sensors.altitude or 0) / 5))))
    disp.setCursorPos(1, 4)
    disp.write(string.format("PITCH: %+4.1f  ROLL:  %+4.1f", sensors.pitch or 0, sensors.roll or 0))
    disp.setCursorPos(1, 5)
    local vs = sensors.vertical_speed or 0
    disp.write(string.format("V/S:  %4.1f %s  PRESSURE: %.2f",
        math.abs(vs), vs > 0 and "^" or (vs < 0 and "v" or "-"), sensors.air_pressure or 1))
    disp.setCursorPos(1, 6)
    disp.write(string.format("TGT: SPD=%-4s ALT=%-4s",
        targets.airspeed or "---", targets.altitude or "---"))
    disp.update()
end

return Hardware
