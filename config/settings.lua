-- Configuration for Aeronautics Flight Controller v1.3.1-patch
-- PATCH: roll PID strengthened, throttle rate limit added

local CONFIG = {}
CONFIG.version = "1.3.1"

-- Peripheral sides
CONFIG.peripherals = {
    gimbal          = "left",
    altitude        = nil,
    velocity        = nil,
    navigation      = nil,
    elevator        = "Create_SequencedGearshift_12",
    aileron_left    = "Create_SequencedGearshift_6",
    aileron_right   = "Create_SequencedGearshift_7",
    rudder          = "Create_SequencedGearshift_13",
    throttle        = "Create_RotationSpeedController_1",
    display         = nil,
}

CONFIG.limits = {
    max_throttle_rpm    = 256,
    min_throttle_rpm    = 0,
    idle_throttle_rpm   = 64,
    max_elevator_angle  = 45,
    max_aileron_angle   = 25,
    max_rudder_angle    = 45,
    gearshift_speed_mod = 1,
}

CONFIG.pid = {
    throttle = {
        kp           = 0.8,
        ki           = 0.02,
        kd           = 0.15,
        integral_max = 200,
        output_min   = -256,
        output_max   = 256,
    },
    pitch = {
        kp           = 1.2,     -- PATCH: 1.0 -> 1.2, faster elevator response
        ki           = 0.03,
        kd           = 0.5,     -- PATCH: 0.4 -> 0.5, more damping
        integral_max = 80,
        output_min   = -45,
        output_max   = 45,
    },
    -- PATCH: roll gains significantly increased
    -- Old: kp=0.6 ki=0.015 kd=0.25
    -- Roll was too sluggish to counter bank during climb; aircraft entered spiral
    roll = {
        kp           = 1.2,     -- PATCH: 0.6 -> 1.2
        ki           = 0.02,    -- PATCH: 0.015 -> 0.02
        kd           = 0.5,     -- PATCH: 0.25 -> 0.5
        integral_max = 40,
        output_min   = -25,
        output_max   = 25,
    },
    yaw = {
        kp           = 3.0,
        ki           = 0.0,
        kd           = 0.0,
        integral_max = 10,
        output_min   = -20,
        output_max   = 20,
    },
    altitude = {
        kp           = 0.2,     -- PATCH: 0.15 -> 0.2
        ki           = 0.005,
        kd           = 0.08,    -- PATCH: 0.05 -> 0.08
        integral_max = 100,
        output_min   = -10,
        output_max   = 10,
    },
}

CONFIG.loop = {
    update_rate  = 0.05,
    display_rate = 0.5,
}

CONFIG.defaults = {
    target_airspeed = 128,
    target_altitude = 80,
    target_roll     = 0,
    target_heading  = 0,
}

CONFIG.safety = {
    max_pitch_angle         = 15,
    max_roll_angle          = 30,
    stall_speed             = 32,
    ground_proximity_alt    = 10,
    auto_level_on_emergency = true,
}

-- PATCH: separate rate limits for throttle vs surfaces
-- throttle doesn't need mechanical protection like gearshifts do
CONFIG.rate_limits = {
    surface_deg_per_sec  = 30,   -- elevator/aileron/rudder (degrees/s)
    throttle_rpm_per_sec = 256,  -- PATCH: effectively unlimited (full range in 1s)
}

return CONFIG
