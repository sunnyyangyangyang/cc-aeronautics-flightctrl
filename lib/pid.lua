-- PID Controller Library for Aeronautics Flight Control
-- Implements standard PID with anti-windup and output clamping

local PID = {}
PID.__index = PID

--- Create a new PID controller
-- @param kp Proportional gain
-- @param ki Integral gain
-- @param kd Derivative gain
-- @param integral_max Maximum integral value (anti-windup)
-- @param output_min Minimum output clamp
-- @param output_max Maximum output clamp
-- @return PID controller instance
function PID.new(kp, ki, kd, integral_max, output_min, output_max)
    local self = setmetatable({}, PID)
    self.kp = kp or 0
    self.ki = ki or 0
    self.kd = kd or 0
    self.integral_max = integral_max or 100
    self.output_min = output_min or -256
    self.output_max = output_max or 256

    -- Internal state
    self.integral = 0
    self.prev_error = 0
    self.prev_time = nil
    self.prev_derivative = 0

    return self
end

--- Update PID with new error value
-- @param error Current error (setpoint - measured)
-- @param dt Time delta in seconds (optional, defaults to 0.05)
-- @return Clamped PID output value
function PID:update(error, dt)
    if not dt or dt <= 0 then
        dt = 0.05
    end

    -- Clamp error to prevent spikes
    if error > 1000 then error = 1000 end
    if error < -1000 then error = -1000 end

    -- Integral with anti-windup
    self.integral = self.integral + error * dt
    if self.integral > self.integral_max then
        self.integral = self.integral_max
    elseif self.integral < -self.integral_max then
        self.integral = -self.integral_max
    end

    -- Derivative with noise filtering (simple low-pass)
    local raw_derivative = (error - self.prev_error) / dt
    local derivative = 0.7 * raw_derivative + 0.3 * self.prev_derivative
    self.prev_derivative = derivative

    -- PID calculation
    local output = self.kp * error
                 + self.ki * self.integral
                 + self.kd * derivative

    -- Clamp output
    if output > self.output_max then output = self.output_max end
    if output < self.output_min then output = self.output_min end

    -- Update state
    self.prev_error = error
    self.prev_time = os.clock()

    return output
end

--- Reset PID state (use when switching modes or after large disturbance)
function PID:reset()
    self.integral = 0
    self.prev_error = 0
    self.prev_derivative = 0
    self.prev_time = nil
end

--- Set PID gains dynamically
-- @param kp New proportional gain
-- @param ki New integral gain (optional)
-- @param kd New derivative gain (optional)
function PID:setGains(kp, ki, kd)
    if kp then self.kp = kp end
    if ki then self.ki = ki end
    if kd then self.kd = kd end
end

--- Get current internal state (for debugging)
-- @return table with integral, prev_error, derivative
function PID:getState()
    return {
        integral = self.integral,
        prev_error = self.prev_error,
        derivative = self.prev_derivative,
    }
end

return PID
