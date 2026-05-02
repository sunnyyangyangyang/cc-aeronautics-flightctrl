-- Hardware Abstraction Layer for Aeronautics Flight Control
-- Wraps all peripheral interactions for sensors and actuators

local Hardware = {}
Hardware.__index = Hardware

--- Initialize hardware layer
-- @param config Configuration table with peripheral sides
-- @return Hardware instance
function Hardware.new(config)
    local self = setmetatable({}, Hardware)
    self.config = config or {}

    -- Raw peripherals
    self.peripherals = {}
    -- Parsed sensor data
    self.sensors = {
        pitch = 0,               -- Pitch angle (degrees)
        roll = 0,                -- Roll angle (degrees)
        altitude = 0,            -- Altitude (blocks)
        airspeed = 0,            -- Forward airspeed
        air_pressure = 1.0,      -- Air pressure (0-1)
        heading = 0,             -- Heading relative to target (degrees)
        vertical_speed = 0,      -- Vertical speed (blocks/s)
        -- Angular velocity from CC: Sable sublevel API
        pitch_rate = 0,
        roll_rate = 0,
        yaw_rate = 0,
    }
 -- Previous altitude for vertical speed calculation
    self.prev_altitude = 0
    self.prev_time = 0

    -- Track current control surface angles for delta-based rotation
    -- Sequenced Gearshift rotate(angle) is RELATIVE ("rotates BY angle"), not absolute!
    -- angle must be a positive integer, modifier must be integer in [-2..2]
    self.surface_angles = {
        elevator = 0,
        aileron_left = 0,
        aileron_right = 0,
        rudder = 0,
    }

    return self
end

--- Initialize all peripherals
-- Call this once at startup
function Hardware:initPeripherals()
    local sides = self.config.peripherals or {}

    -- Sensors
    if sides.gimbal then
        self.peripherals.gimbal = peripheral.wrap(sides.gimbal)
    end
    if sides.altitude then
        self.peripherals.altitude = peripheral.wrap(sides.altitude)
    end
    if sides.velocity then
        self.peripherals.velocity = peripheral.wrap(sides.velocity)
    end
    if sides.navigation then
        self.peripherals.navigation = peripheral.wrap(sides.navigation)
    end

    -- Actuators (Sequenced Gearshift for control surfaces)
    if sides.elevator then
        self.peripherals.elevator = peripheral.wrap(sides.elevator)
    end
    if sides.aileron_left then
        self.peripherals.aileron_left = peripheral.wrap(sides.aileron_left)
    end
    if sides.aileron_right then
        self.peripherals.aileron_right = peripheral.wrap(sides.aileron_right)
    end
    if sides.rudder then
        self.peripherals.rudder = peripheral.wrap(sides.rudder)
    end

    -- Throttle (Rotation Speed Controller)
    if sides.throttle then
        self.peripherals.throttle = peripheral.wrap(sides.throttle)
    end

    -- Display
    if sides.display then
        self.peripherals.display = peripheral.wrap(sides.display)
    end

    -- Print detailed status for each peripheral
     local sensor_count = 0
     local actuator_count = 0
     local actuator_names = {}
     local sensor_names = {}
     local missing_list = {}

     -- Define which keys are sensors vs actuators
     local sensor_keys = {"gimbal", "altitude", "velocity", "navigation"}
     local actuator_keys = {"elevator", "aileron_left", "aileron_right", "rudder", "throttle", "display"}

     for _, name in ipairs(sensor_keys) do
         local side = (self.config.peripherals or {})[name]
         if not side then
             table.insert(missing_list, name .. "(not configured)")
         elseif peripheral.isPresent(side) then
             local ptype = peripheral.getType(side) or "unknown"
             print("[HW] OK   " .. name .. " -> " .. side .. " (" .. ptype .. ")")
             sensor_count = sensor_count + 1
             table.insert(sensor_names, name)
         else
             table.insert(missing_list, name .. "(" .. side .. ")")
         end
     end

     for _, name in ipairs(actuator_keys) do
         local side = (self.config.peripherals or {})[name]
         if not side then
             -- Only warn for critical actuators
             if name == "elevator" or name == "throttle" then
                 table.insert(missing_list, name .. "(not configured)")
             end
         elseif peripheral.isPresent(side) then
             local ptype = peripheral.getType(side) or "unknown"
             print("[HW] OK   " .. name .. " -> " .. side .. " (" .. ptype .. ")")
             actuator_count = actuator_count + 1
             table.insert(actuator_names, name)
         else
             table.insert(missing_list, name .. "(" .. side .. ")")
         end
     end

     print("[HW] Summary: " .. sensor_count .. " sensors, " .. actuator_count .. " actuators ("
           .. table.concat(actuator_names, ", ") or "none" .. ")")

     if #missing_list > 0 then
         print("[HW] WARNING - Missing: " .. table.concat(missing_list, ", "))
     end
end

--- Read all sensor data
-- Combines CC: Sable sublevel API with peripheral sensors
function Hardware:readSensors()
    local now = os.clock()
    local dt = now - self.prev_time
    if dt <= 0 then dt = 0.05 end
    self.prev_time = now

    -- Try CC: Sable sublevel API (works on assembled contraptions)
    -- Always read sublevel data: altitude/speed are PRIMARY sources
    if sublevel and sublevel.isInPlotGrid and sublevel.isInPlotGrid() then
        -- Altitude from pose (PRIMARY)
        if sublevel.getLogicalPose then
            local pose = sublevel.getLogicalPose()
            if pose and pose.y then
                local alt = pose.y
                self.sensors.vertical_speed = (alt - self.prev_altitude) / dt
                self.prev_altitude = alt
                self.sensors.altitude = alt
            end
        end

        -- Linear velocity for airspeed (PRIMARY)
        if sublevel.getLinearVelocity then
            local linVel = sublevel.getLinearVelocity()
            local speed = math.sqrt(
                (linVel.x or 0)^2 + (linVel.y or 0)^2 + (linVel.z or 0)^2
            )
            self.sensors.airspeed = speed
        end

        -- Angular velocity for rate-based PID control
        if sublevel.getAngularVelocity then
            local angVel = sublevel.getAngularVelocity()
            self.sensors.pitch_rate = angVel.x or 0
            self.sensors.roll_rate = angVel.z or 0
            self.sensors.yaw_rate = angVel.y or 0
        end
    end

    -- Read gimbal sensor (attitude)
    if self.peripherals.gimbal and self.peripherals.gimbal.getAngles then
        local angles = self.peripherals.gimbal.getAngles()
        self.sensors.pitch = angles[1] or 0
        self.sensors.roll = angles[2] or 0
    end

    -- Fallback: Altitude Sensor peripheral (only if sublevel not available)
    if self.sensors.altitude == 0 and self.peripherals.altitude and self.peripherals.altitude.getHeight then
        local alt = self.peripherals.altitude.getHeight()
        self.sensors.vertical_speed = (alt - self.prev_altitude) / dt
        self.prev_altitude = alt
        self.sensors.altitude = alt
    end

    -- Air pressure from altitude sensor
    if self.peripherals.altitude and self.peripherals.altitude.getAirPressure then
        self.sensors.air_pressure = self.peripherals.altitude.getAirPressure()
    end

    -- Fallback: Velocity Sensor peripheral (only if sublevel not available)
    if self.sensors.airspeed == 0 and self.peripherals.velocity and self.peripherals.velocity.getVelocity then
        self.sensors.airspeed = self.peripherals.velocity.getVelocity()
    end

    -- Read navigation table (heading) - getRelativeAngle() returns Float (can be nil)
    if self.peripherals.navigation and self.peripherals.navigation.getRelativeAngle then
        local heading = self.peripherals.navigation.getRelativeAngle()
        self.sensors.heading = heading or 0
    end

    return self.sensors
end

--- Read data from CC: Sable sublevel API
-- Provides angular velocity and precise pose data
function Hardware:readSublevelData()
    -- Angular velocity for rate-based PID control
    if sublevel.getAngularVelocity then
        local angVel = sublevel.getAngularVelocity()
        self.sensors.pitch_rate = angVel.x or 0
        self.sensors.roll_rate = angVel.z or 0
        self.sensors.yaw_rate = angVel.y or 0
    end

    -- Linear velocity for airspeed fallback
    if sublevel.getLinearVelocity then
        local linVel = sublevel.getLinearVelocity()
        -- Calculate speed magnitude
        local speed = math.sqrt(
            (linVel.x or 0)^2 + (linVel.y or 0)^2 + (linVel.z or 0)^2
        )
        -- Only use if velocity sensor is not available
        if not self.peripherals.velocity then
            self.sensors.airspeed = speed
        end
    end

    -- Altitude from pose (fallback)
    if sublevel.getLogicalPose then
        local pose = sublevel.getLogicalPose()
        if pose and pose.y then
            if not self.peripherals.altitude then
                self.sensors.altitude = pose.y
            end
        end
    end
end

--- Set throttle (propeller RPM)
-- @param rpm Target RPM (-256 to 256)
function Hardware:setThrottle(rpm)
    if not self.peripherals.throttle then return end

    -- Clamp to limits
    local max_rpm = self.config.limits.max_throttle_rpm or 256
    local min_rpm = self.config.limits.min_throttle_rpm or 0
    rpm = math.max(min_rpm, math.min(max_rpm, rpm))

    if self.peripherals.throttle.setTargetSpeed then
        self.peripherals.throttle.setTargetSpeed(math.floor(rpm))
    end
end

--- Set elevator deflection (delta-based: rotate BY difference from current angle)
-- @param angle Target angle in degrees (positive = nose up)
function Hardware:setElevator(angle)
    if not self.peripherals.elevator then return end

    -- Clamp to limits
    local max_angle = self.config.limits.max_elevator_angle or 30
    angle = math.max(-max_angle, math.min(max_angle, angle))

    -- Calculate delta (Sequenced Gearshift rotate() is RELATIVE)
    local current = self.surface_angles.elevator
    local delta = angle - current
    if math.abs(delta) < 0.1 then return end  -- Lowered threshold for finer control

    -- rotate(angle, modifier): angle must be positive integer, modifier integer [-2..2]
    local rot_angle = math.max(1, math.floor(math.abs(delta) + 0.5))
    local modifier = delta > 0 and 1 or -1
    local speed_mod = self.config.limits.gearshift_speed_mod or 1
    -- Clamp modifier to valid range [-2..2], ensure integer
    local final_mod = math.floor(math.max(-2, math.min(2, modifier * math.abs(speed_mod))))

    if self.peripherals.elevator.rotate then
        self.peripherals.elevator.rotate(rot_angle, final_mod)
    end
    self.surface_angles.elevator = angle
end

--- Set aileron deflection (differential, delta-based)
-- @param angle Target aileron angle (positive = right wing down = roll right)
-- Note: Works with single aileron (only left or only right connected)
function Hardware:setAilerons(angle)
    -- Allow single-side operation: only return if BOTH are missing
    if not self.peripherals.aileron_left and not self.peripherals.aileron_right then return end

    -- Clamp to limits
    local max_angle = self.config.limits.max_aileron_angle or 25
    angle = math.max(-max_angle, math.min(max_angle, angle))

    local speed_mod = self.config.limits.gearshift_speed_mod or 1
    -- Ensure integer for Create Java API
    local final_mod = math.floor(math.max(-2, math.min(2, math.abs(speed_mod))))

    -- Differential: left and right ailerons move opposite
    -- Left aileron
    if self.peripherals.aileron_left then
        local left_delta = -angle - self.surface_angles.aileron_left
        if math.abs(left_delta) >= 0.1 then
            local rot_angle = math.max(1, math.floor(math.abs(left_delta) + 0.5))
            local mod = left_delta > 0 and final_mod or (-final_mod)
            if self.peripherals.aileron_left.rotate then
                self.peripherals.aileron_left.rotate(rot_angle, mod)
            end
        end
    end

    -- Right aileron
    if self.peripherals.aileron_right then
        local right_delta = angle - self.surface_angles.aileron_right
        if math.abs(right_delta) >= 0.1 then
            local rot_angle = math.max(1, math.floor(math.abs(right_delta) + 0.5))
            local mod = right_delta > 0 and final_mod or (-final_mod)
            if self.peripherals.aileron_right.rotate then
                self.peripherals.aileron_right.rotate(rot_angle, mod)
            end
        end
    end

    self.surface_angles.aileron_left = -angle
    self.surface_angles.aileron_right = angle
end

--- Set rudder deflection (delta-based)
-- @param angle Target angle in degrees (positive = yaw right)
function Hardware:setRudder(angle)
    if not self.peripherals.rudder then return end

    -- Clamp to limits
    local max_angle = self.config.limits.max_rudder_angle or 20
    angle = math.max(-max_angle, math.min(max_angle, angle))

    -- Calculate delta
    local delta = angle - self.surface_angles.rudder
    if math.abs(delta) < 0.1 then return end

    local rot_angle = math.max(1, math.floor(math.abs(delta) + 0.5))
    local modifier = delta > 0 and 1 or -1
    local speed_mod = self.config.limits.gearshift_speed_mod or 1
    -- Ensure integer for Create Java API
    local final_mod = math.floor(math.max(-2, math.min(2, modifier * math.abs(speed_mod))))

    if self.peripherals.rudder.rotate then
        self.peripherals.rudder.rotate(rot_angle, final_mod)
    end
    self.surface_angles.rudder = angle
end

--- Neutralize all control surfaces (center everything)
function Hardware:neutralize()
    self:setElevator(0)
    self:setAilerons(0)
    self:setRudder(0)
end

--- Reset surface angle tracking (call after assembly or if angles drift)
function Hardware:resetSurfaceTracking()
    self.surface_angles.elevator = 0
    self.surface_angles.aileron_left = 0
    self.surface_angles.aileron_right = 0
    self.surface_angles.rudder = 0
end

--- Stop throttle
function Hardware:stopThrottle()
    self:setThrottle(0)
end

--- Update display with current sensor data
-- @param sensors Current sensor readings
-- @param mode Current flight mode
-- @param targets Current target values
function Hardware:updateDisplay(sensors, mode, targets)
    if not self.peripherals.display then return end

    local disp = self.peripherals.display

    -- Clear and write HUD
    disp.clear()

    -- Line 1: Mode and heading
    disp.setCursorPos(1, 1)
    disp.write(string.format("MODE: %-8s HDG: %4.1f", mode, sensors.heading or 0))

    -- Line 2: Airspeed
    disp.setCursorPos(1, 2)
    local speed_bar = string.rep("#", math.floor((sensors.airspeed or 0) / 10))
    disp.write(string.format("SPD:  %4.1f  %-16s", sensors.airspeed or 0, speed_bar))

    -- Line 3: Altitude
    disp.setCursorPos(1, 3)
    local alt_bar = string.rep("|", math.floor((sensors.altitude or 0) / 5))
    disp.write(string.format("ALT:  %4.1f  %-16s", sensors.altitude or 0, alt_bar))

    -- Line 4: Attitude indicator (simplified)
    disp.setCursorPos(1, 4)
    local pitch_str = string.format("PITCH: %+4.1f", sensors.pitch or 0)
    local roll_str = string.format("ROLL:  %+4.1f", sensors.roll or 0)
    disp.write(pitch_str .. "  " .. roll_str)

    -- Line 5: Vertical speed
    disp.setCursorPos(1, 5)
    local vs = sensors.vertical_speed or 0
    local vs_symbol = vs > 0 and "^" or (vs < 0 and "v" or "-")
    disp.write(string.format("V/S:  %4.1f %s  PRESSURE: %.2f", math.abs(vs), vs_symbol, sensors.air_pressure or 1))

    -- Line 6: Targets
    disp.setCursorPos(1, 6)
    disp.write(string.format("TGT: SPD=%-4s ALT=%-4s",
        targets.airspeed or "---",
        targets.altitude or "---"))

    -- Push to display
    disp.update()
end

return Hardware
