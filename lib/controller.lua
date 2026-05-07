-- lib/controller.lua  v1.3.1-patch
-- PATCH list:
--   [1] Separate throttle rate limit (no longer choked by surface limiter)
--   [2] Roll authority: clamp removed during level-flight enforcement
--   [3] Roll leveling guard: if |roll| > 20° and low speed, override aileron fully

local PID = require("lib/pid")
local Controller = {}
Controller.__index = Controller

function Controller.new(config)
    local self = setmetatable({}, Controller)
    self.config = config or {}

    self.pid_altitude = PID.new(
        config.pid.altitude.kp, config.pid.altitude.ki, config.pid.altitude.kd,
        config.pid.altitude.integral_max,
        config.pid.altitude.output_min, config.pid.altitude.output_max)

    self.pid_throttle = PID.new(
        config.pid.throttle.kp, config.pid.throttle.ki, config.pid.throttle.kd,
        config.pid.throttle.integral_max,
        config.pid.throttle.output_min, config.pid.throttle.output_max)

    self.pid_pitch = PID.new(
        config.pid.pitch.kp, config.pid.pitch.ki, config.pid.pitch.kd,
        config.pid.pitch.integral_max,
        config.pid.pitch.output_min, config.pid.pitch.output_max)

    self.pid_roll = PID.new(
        config.pid.roll.kp, config.pid.roll.ki, config.pid.roll.kd,
        config.pid.roll.integral_max,
        config.pid.roll.output_min, config.pid.roll.output_max)

    self.pid_yaw = PID.new(
        config.pid.yaw.kp, config.pid.yaw.ki, config.pid.yaw.kd,
        config.pid.yaw.integral_max,
        config.pid.yaw.output_min, config.pid.yaw.output_max)

    -- Sensor filters
    self.filtered_airspeed = 0
    self.airspeed_alpha    = 0.1
    self.max_airspeed      = 500

    -- Output rate limiters
    self.prev_elevator = 0
    self.prev_aileron  = 0
    self.prev_rudder   = 0
    self.prev_throttle = 0

    -- PATCH: read separate rate limits from config, with sane fallbacks
    local rl = (config.rate_limits or {})
    self.surface_rate_limit  = rl.surface_deg_per_sec  or 30
    self.throttle_rate_limit = rl.throttle_rpm_per_sec or 256  -- PATCH: was hardcoded 30

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
    self.prev_aileron  = 0
    self.prev_rudder   = 0
    self.prev_throttle = 0
end

function Controller:filterAirspeed(raw)
    if raw > self.max_airspeed then raw = self.max_airspeed end
    raw = math.max(0, raw)
    self.filtered_airspeed = (1 - self.airspeed_alpha) * self.filtered_airspeed
                           + self.airspeed_alpha * raw
    return self.filtered_airspeed
end

function Controller:rateLimit(current, previous, dt, limit)
    local maxDelta = limit * dt
    local delta = current - previous
    if     delta >  maxDelta then return previous + maxDelta
    elseif delta < -maxDelta then return previous - maxDelta
    else                          return current
    end
end

--- Main control function
function Controller:update(targets, sensors, dt)
    if not dt or dt <= 0 then dt = 0.05 end

    local spd = self:filterAirspeed(sensors.airspeed)

    -- Outer loop: altitude -> pitch target
    local altError    = targets.altitude - sensors.altitude
    local pitchTarget = self.pid_altitude:update(altError, dt)
    pitchTarget = math.max(-15, math.min(15, pitchTarget))

    -- Inner loop: pitch -> elevator
    local pitchError  = pitchTarget - sensors.pitch
    local elevatorCmd = self.pid_pitch:update(pitchError, dt)

    -- Inner loop: roll -> aileron
    local rollError = targets.roll - sensors.roll
    rollError = math.max(-30, math.min(30, rollError))
    local aileronCmd = self.pid_roll:update(rollError, dt)

    -- PATCH: roll leveling guard
    -- If bank angle is dangerously large and we're slow, slam ailerons to full
    -- regardless of PID output. Prevents spiral-dive on climbout.
    local absRoll = math.abs(sensors.roll)
    if absRoll > 20 then
        local maxAil = self.config.limits.max_aileron_angle or 25
        -- Direction: if rolled right (positive), we need negative aileron and vice versa
        local guardCmd = -math.sign_or(sensors.roll, 1) * maxAil
        -- Blend: the further past 20° we are, the more authority we take
        local blend = math.min(1.0, (absRoll - 20) / 10)   -- 0 at 20°, 1.0 at 30°
        aileronCmd = aileronCmd * (1 - blend) + guardCmd * blend
    end

    -- Inner loop: yaw damping / heading hold
    local yawInput
    if sensors.yaw_rate and sensors.yaw_rate ~= 0 then
        yawInput = -sensors.yaw_rate
    else
        local headingError = targets.heading - sensors.heading
        if headingError >  180 then headingError = headingError - 360 end
        if headingError < -180 then headingError = headingError + 360 end
        yawInput = headingError
    end
    local rudderCmd = self.pid_yaw:update(yawInput, dt)

    -- Outer loop: airspeed -> throttle
    local speedError  = targets.airspeed - spd
    local throttleCmd = self.pid_throttle:update(speedError, dt)

    -- Stall protection
    if spd < self.config.safety.stall_speed and spd > 0 then
        elevatorCmd = math.max(elevatorCmd, 10)
        throttleCmd = math.max(throttleCmd, 192)
    end

    -- Ground proximity
    if sensors.altitude < self.config.safety.ground_proximity_alt
       and sensors.altitude > 0 then
        if sensors.vertical_speed and sensors.vertical_speed < -2 then
            elevatorCmd = math.max(elevatorCmd, 15)
        end
    end

    -- Clamp to hardware limits
    local lim = self.config.limits
    elevatorCmd = math.max(-lim.max_elevator_angle, math.min(lim.max_elevator_angle, elevatorCmd))
    aileronCmd  = math.max(-lim.max_aileron_angle,  math.min(lim.max_aileron_angle,  aileronCmd))
    rudderCmd   = math.max(-lim.max_rudder_angle,   math.min(lim.max_rudder_angle,   rudderCmd))
    throttleCmd = math.max(lim.min_throttle_rpm,    math.min(lim.max_throttle_rpm,   throttleCmd))

    -- PATCH: surfaces use surface_rate_limit, throttle uses throttle_rate_limit
    elevatorCmd = self:rateLimit(elevatorCmd, self.prev_elevator, dt, self.surface_rate_limit)
    aileronCmd  = self:rateLimit(aileronCmd,  self.prev_aileron,  dt, self.surface_rate_limit)
    rudderCmd   = self:rateLimit(rudderCmd,   self.prev_rudder,   dt, self.surface_rate_limit)
    throttleCmd = self:rateLimit(throttleCmd, self.prev_throttle, dt, self.throttle_rate_limit)  -- PATCH

    self.prev_elevator = elevatorCmd
    self.prev_aileron  = aileronCmd
    self.prev_rudder   = rudderCmd
    self.prev_throttle = throttleCmd

    return {
        elevator     = elevatorCmd,
        aileron      = aileronCmd,
        rudder       = rudderCmd,
        throttle     = throttleCmd,
        pitchTarget  = pitchTarget,
        pitchError   = pitchError,
        rollError    = rollError,
        altError     = altError,
        speedError   = speedError,
        filteredSpeed = spd,
    }
end

-- PATCH: helper (Lua has no math.sign)
function math.sign_or(x, default)
    if x > 0 then return 1
    elseif x < 0 then return -1
    else return default or 0
    end
end

return Controller
