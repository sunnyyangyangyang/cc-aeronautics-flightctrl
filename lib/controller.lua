local PID = require("lib/pid")
local Controller = {}
Controller.__index = Controller

function Controller.new(config)
    local self = setmetatable({}, Controller)
    self.config = config or {}

    -- Outer loop: altitude -> pitch target
    self.pid_altitude = PID.new(
        config.pid.altitude.kp,
        config.pid.altitude.ki,
        config.pid.altitude.kd,
        config.pid.altitude.integral_max,
        config.pid.altitude.output_min,
        config.pid.altitude.output_max
    )

    -- Outer loop: airspeed -> throttle
    self.pid_throttle = PID.new(
        config.pid.throttle.kp,
        config.pid.throttle.ki,
        config.pid.throttle.kd,
        config.pid.throttle.integral_max,
        config.pid.throttle.output_min,
        config.pid.throttle.output_max
    )

    -- Inner loop: pitch angle -> elevator
    self.pid_pitch = PID.new(
        config.pid.pitch.kp,
        config.pid.pitch.ki,
        config.pid.pitch.kd,
        config.pid.pitch.integral_max,
        config.pid.pitch.output_min,
        config.pid.pitch.output_max
    )

    -- Inner loop: roll angle -> aileron
    self.pid_roll = PID.new(
        config.pid.roll.kp,
        config.pid.roll.ki,
        config.pid.roll.kd,
        config.pid.roll.integral_max,
        config.pid.roll.output_min,
        config.pid.roll.output_max
    )

    -- Inner loop: heading -> rudder
    self.pid_yaw = PID.new(
        config.pid.yaw.kp,
        config.pid.yaw.ki,
        config.pid.yaw.kd,
        config.pid.yaw.integral_max,
        config.pid.yaw.output_min,
        config.pid.yaw.output_max
    )

    -- Sensor filters
    self.filtered_airspeed = 0
    self.airspeed_alpha = 0.3
    self.max_airspeed = 500

    -- Output rate limiters (degrees/s)
    self.prev_elevator = 0
    self.prev_aileron = 0
    self.prev_rudder = 0
    self.prev_throttle = 0
    self.prev_lifter = 0
    self.output_rate_limit = 30

    return self
end

function Controller:reset()
    self.pid_altitude:reset()
    self.pid_throttle:reset()
    self.pid_pitch:reset()
    self.pid_roll:reset()
    self.pid_yaw:reset()
    self.filtered_airspeed = 0
    self.prev_elevator = 0
    self.prev_aileron = 0
    self.prev_rudder = 0
    self.prev_throttle = 0
    self.prev_lifter = 0
end

function Controller:filterAirspeed(raw)
    if raw > self.max_airspeed then
        raw = self.max_airspeed
    end
    raw = math.max(0, raw)
    self.filtered_airspeed = (1 - self.airspeed_alpha) * self.filtered_airspeed + self.airspeed_alpha * raw
    return self.filtered_airspeed
end

function Controller:rateLimit(current, previous, dt, limit)
    local maxDelta = limit * dt
    local delta = current - previous
    if delta > maxDelta then
        return previous + maxDelta
    elseif delta < -maxDelta then
        return previous - maxDelta
    end
    return current
end

--- Main control function
--- Input: targets {altitude, airspeed, roll, heading}, sensors {pitch, roll, altitude, airspeed, heading}
--- Output: {elevator, aileron, rudder, throttle, lifter, ...intermediates for log}
function Controller:update(targets, sensors, dt)
    if not dt or dt <= 0 then dt = 0.05 end

    local spd = self:filterAirspeed(sensors.airspeed)

    -- Outer loop: altitude -> pitch target
    local altError = targets.altitude - sensors.altitude
    local pitchTarget = self.pid_altitude:update(altError, dt)
    pitchTarget = math.max(-15, math.min(15, pitchTarget))

    -- Inner loop: pitch target -> elevator
    local pitchError = pitchTarget - sensors.pitch
    local elevatorCmd = self.pid_pitch:update(pitchError, dt)

    -- Inner loop: roll target -> aileron
    local rollError = targets.roll - sensors.roll
    rollError = math.max(-30, math.min(30, rollError))
    local aileronCmd = self.pid_roll:update(rollError, dt)

    -- Inner loop: heading -> rudder
    local headingError = targets.heading - sensors.heading
    if headingError > 180 then headingError = headingError - 360 end
    if headingError < -180 then headingError = headingError + 360 end
    local rudderCmd = self.pid_yaw:update(headingError, dt)

    -- Outer loop: airspeed -> throttle
    local speedError = targets.airspeed - spd
    local throttleCmd = self.pid_throttle:update(speedError, dt)

    -- Tail lifter: tracks pitch error for additional pitch authority
    -- Lifter provides same-sign deflection as elevator for coordinated pitch control
    local lifterCmd = elevatorCmd * 0.8
    local max_lifter = self.config.limits.max_lifter_angle or 45
    lifterCmd = math.max(-max_lifter, math.min(max_lifter, lifterCmd))

    -- Stall protection
    if spd < self.config.safety.stall_speed and spd > 0 then
        elevatorCmd = math.min(elevatorCmd, -10)
        lifterCmd = math.min(lifterCmd, -8)
        throttleCmd = math.max(throttleCmd, 192)
    end

    -- Ground proximity
    if sensors.altitude < self.config.safety.ground_proximity_alt and sensors.altitude > 0 then
        if sensors.vertical_speed and sensors.vertical_speed < -2 then
            elevatorCmd = math.max(elevatorCmd, 15)
            lifterCmd = math.max(lifterCmd, 12)
        end
    end

    -- Clamp to limits
    elevatorCmd = math.max(-self.config.limits.max_elevator_angle, math.min(self.config.limits.max_elevator_angle, elevatorCmd))
    aileronCmd = math.max(-self.config.limits.max_aileron_angle, math.min(self.config.limits.max_aileron_angle, aileronCmd))
    rudderCmd = math.max(-self.config.limits.max_rudder_angle, math.min(self.config.limits.max_rudder_angle, rudderCmd))
    throttleCmd = math.max(self.config.limits.min_throttle_rpm, math.min(self.config.limits.max_throttle_rpm, throttleCmd))
    lifterCmd = math.max(-max_lifter, math.min(max_lifter, lifterCmd))

    -- Rate limit outputs
    elevatorCmd = self:rateLimit(elevatorCmd, self.prev_elevator, dt, self.output_rate_limit)
    aileronCmd = self:rateLimit(aileronCmd, self.prev_aileron, dt, self.output_rate_limit)
    rudderCmd = self:rateLimit(rudderCmd, self.prev_rudder, dt, self.output_rate_limit)
    throttleCmd = self:rateLimit(throttleCmd, self.prev_throttle, dt, self.output_rate_limit)
    lifterCmd = self:rateLimit(lifterCmd, self.prev_lifter, dt, self.output_rate_limit)

    self.prev_elevator = elevatorCmd
    self.prev_aileron = aileronCmd
    self.prev_rudder = rudderCmd
    self.prev_throttle = throttleCmd
    self.prev_lifter = lifterCmd

    return {
        -- Outputs
        elevator = elevatorCmd,
        aileron = aileronCmd,
        rudder = rudderCmd,
        throttle = throttleCmd,
        lifter = lifterCmd,
        -- Intermediates for logging
        pitchTarget = pitchTarget,
        pitchError = pitchError,
        rollError = rollError,
        altError = altError,
        speedError = speedError,
        filteredSpeed = spd,
    }
end

return Controller
