-- Configuration for Aeronautics Flight Controller
-- Adjust these values for your specific aircraft

local CONFIG = {}

-- Peripheral sides (attach peripherals to top/bottom to avoid Sable #638 bug)
CONFIG.peripherals = {
    gimbal = "left",           -- Gimbal Sensor (attitude: pitch + roll)
    altitude = nil,            -- Altitude Sensor
    velocity = nil,           -- Velocity Sensor (facing forward)
    navigation = nil,        -- Navigation Table (optional, for heading)
    -- Control actuators via Sequenced Gearshift
    elevator = "Create_SequencedGearshift_10",          -- Sequenced Gearshift -> Elevator Swivel Bearing
    aileron_left = "Create_SequencedGearshift_6",       -- Sequenced Gearshift -> Left Aileron
    aileron_right = "Create_SequencedGearshift_7",    -- Sequenced Gearshift -> Right Aileron
    rudder = "right",              -- Sequenced Gearshift -> Rudder (optional)
    -- Throttle control
    throttle = "Create_RotationSpeedController_0",           -- Rotation Speed Controller for propeller
    -- Display
    display = nil,           -- Display Link or monitor
}

-- Aircraft limits
CONFIG.limits = {
    max_throttle_rpm = 256,      -- Max propeller RPM
    min_throttle_rpm = 0,        -- Min propeller RPM (idle)
    idle_throttle_rpm = 64,      -- Idle/cruise minimum
    max_elevator_angle = 45,     -- Max elevator deflection (degrees), up/down
    max_aileron_angle = 25,      -- Max aileron deflection (degrees)
    max_rudder_angle = 45,       -- Max rudder deflection (degrees), left/right
    gearshift_speed_mod = 1,     -- Sequenced Gearshift speed modifier (-2..2)
}

-- PID tuning parameters
-- Start with these and adjust based on aircraft behavior
CONFIG.pid = {
    -- Throttle PID: controls airspeed
    throttle = {
        kp = 0.8,
        ki = 0.02,
        kd = 0.15,
        integral_max = 200,
        output_min = -256,
        output_max = 256,
    },
    -- Pitch PID: controls altitude via elevator
    pitch = {
        kp = 0.4,
        ki = 0.01,
        kd = 0.2,
        integral_max = 50,
        output_min = -30,
        output_max = 30,
    },
    -- Roll PID: controls bank angle via ailerons
    roll = {
        kp = 0.6,
        ki = 0.015,
        kd = 0.25,
        integral_max = 40,
        output_min = -25,
        output_max = 25,
    },
    -- Yaw PID: controls heading via rudder
    yaw = {
        kp = 0.3,
        ki = 0.005,
        kd = 0.1,
        integral_max = 30,
        output_min = -20,
        output_max = 20,
    },
    -- Altitude PID: outer loop, feeds into pitch target
    altitude = {
        kp = 0.15,
        ki = 0.005,
        kd = 0.05,
        integral_max = 100,
        output_min = -10,
        output_max = 10,
    },
}

-- Control loop timing
CONFIG.loop = {
    update_rate = 0.05,          -- PID update interval (seconds), 20Hz
    display_rate = 0.5,          -- Display update interval (seconds)
}

-- Default flight parameters
CONFIG.defaults = {
    target_airspeed = 128,       -- Default cruise speed (RPM equivalent)
    target_altitude = 80,        -- Default cruise altitude (blocks)
    target_roll = 0,             -- Default bank angle (degrees)
    target_heading = 0,          -- Default heading (degrees)
}

-- Safety
CONFIG.safety = {
    max_pitch_angle = 15,        -- Max allowed pitch (degrees)
    max_roll_angle = 30,         -- Max allowed roll (degrees)
    stall_speed = 32,            -- Minimum safe airspeed
    ground_proximity_alt = 10,   -- Altitude threshold for ground proximity warning
    auto_level_on_emergency = true,
}

return CONFIG
