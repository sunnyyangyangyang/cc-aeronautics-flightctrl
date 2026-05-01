# Aeronautics Flight Controller - API Reference

> All APIs verified against official source code and documentation.
> Last updated: 2026-05-01

---

## Table of Contents

1. [CC: Sable - `sublevel` API](#1-cc-sable---sublevel-api)
2. [CC: Sable - `aero` API](#2-cc-sable---aero-api)
3. [Aeronautics/Simulated - Sensor Peripherals](#3-aeronaticssimulated---sensor-peripherals)
4. [Aeronautics/Simulated - Actuator Peripherals](#4-aeronaticssimulated---actuator-peripherals)
5. [Create CC: Tweaked - Peripherals](#5-create-cc-tweaked---peripherals)
6. [CC: Tweaked Core APIs](#6-cc-tweaked-core-apis)
7. [Critical Notes and Known Issues](#7-critical-notes-and-known-issues)

---

## 1. CC: Sable - `sublevel` API

**Source:** https://techtastic.github.io/CC-Sable/modules/sublevel.html
**Mod:** CC: Sable by TechTastic (https://modrinth.com/mod/cc-sable)
**Availability:** Global `sublevel` table, ONLY when computer is on a Sable Sub-Level (assembled contraption)

> **CRITICAL:** All methods EXCEPT `isInPlotGrid()` will **error** if the computer is not on a Sub-Level. Always check first!

### Functions

| Function | Returns | Description |
|---|---|---|
| `sublevel.isInPlotGrid()` | `boolean` | Check if computer is on a Sub-Level |
| `sublevel.getUniqueId()` | `string` | UUID of the Sub-Level |
| `sublevel.getName()` | `string` | Name of the Sub-Level |
| `sublevel.setName(newName)` | — | Set the Sub-Level name |
| `sublevel.getLogicalPose()` | `table` | Position, orientation (quaternion), scale, rotationPoint |
| `sublevel.getLastPose()` | `table` | Previous frame pose (same format) |
| `sublevel.getVelocity()` | `vector` | Global velocity of the Sub-Level |
| `sublevel.getLinearVelocity()` | `vector` | Linear velocity {x, y, z} |
| `sublevel.getAngularVelocity()` | `vector` | Angular velocity {x=pitchRate, y=yawRate, z=rollRate} |
| `sublevel.getCenterOfMass()` | `vector` | Center of mass position |
| `sublevel.getMass()` | `number` | Mass of the Sub-Level |
| `sublevel.getInverseMass()` | `number` | Inverse mass |
| `sublevel.getInertiaTensor()` | `matrix` | Inertia tensor |
| `sublevel.getInverseInertiaTensor()` | `matrix` | Inverse inertia tensor |

### Usage Pattern

```lua
-- ALWAYS check first!
if sublevel and sublevel.isInPlotGrid and sublevel.isInPlotGrid() then
    local angVel = sublevel.getAngularVelocity()
    print("Pitch rate:", angVel.x, "Roll rate:", angVel.z)
end
```

---

## 2. CC: Sable - `aero` API

**Source:** https://techtastic.github.io/CC-Sable/modules/aero.html
**Availability:** Global `aero` table (always available when CC: Sable is installed)

### Functions

| Function | Returns | Description |
|---|---|---|
| `aero.getAirPressure(position)` | `number` | Air pressure at given position (takes `vector`) |
| `aero.getGravity()` | `vector` | Dimension gravity vector |
| `aero.getMagneticNorth()` | `vector` | Magnetic north vector |
| `aero.getUniversalDrag()` | `number` | Universal drag constant |
| `aero.getRaw()` | `table` | Raw physics data (gravity, pressure, magneticNorth, universalDrag, pressureFunction) |
| `aero.getDefault()` | `table` | Default physics info (used if no JSON config is set) |

---

## 3. Aeronautics/Simulated - Sensor Peripherals

**Source:** Verified from `ComputerCraftPeripherals.java` and individual peripheral source files
**Repository:** https://github.com/Creators-of-Aeronautics/Simulated-Project

### Gimbal Sensor (`gimbal_sensor`) — Attitude Indicator / 水平仪

**Source:** `GimbalSensorPeripheral.java`

| Method | Returns | Description |
|---|---|---|
| `getAngles()` | `{pitchDeg: number, rollDeg: number}` | Pitch (X-axis) and Roll (Z-axis) in **degrees** |
| `getAnglesRad()` | `{pitchRad: number, rollRad: number}` | Same angles in **radians** |

```lua
local gimbal = peripheral.wrap("bottom")
local pitch, roll = table.unpack(gimbal.getAngles())  -- Lua list indexing: [1]=pitch, [2]=roll
-- OR
local angles = gimbal.getAngles()
local pitch = angles[1]
local roll = angles[2]
```

### Altitude Sensor (`altitude_sensor`)

**Source:** `AltitudeSensorPeripheral.java`

| Method | Returns | Description |
|---|---|---|
| `getHeight()` | `float` | Current world height (blocks, Y coordinate) |
| `getAirPressure()` | `double` | Air pressure at current position (0-1 range) |

```lua
local alt = peripheral.wrap("top")
local height = alt.getHeight()       -- e.g., 80.5
local pressure = alt.getAirPressure() -- e.g., 0.85
```

### Velocity Sensor (`velocity_sensor`)

**Source:** `VelocitySensorPeripheral.java`

| Method | Returns | Description |
|---|---|---|
| `getVelocity()` | `float` | Adjusted velocity along the sensor's facing axis |

```lua
local vel = peripheral.wrap("back")
local speed = vel.getVelocity()  -- speed along facing direction
```

### Navigation Table (`navigation_table`)

**Source:** `NavTablePeripheral.java`

| Method | Returns | Description |
|---|---|---|
| `getRelativeAngle()` | `Float` (can be **nil**!) | Angle to navigation target in degrees |
| `getRelativeAngleRad()` | `double` | Same angle in radians |

> **WARNING:** `getRelativeAngle()` returns Java `Float` (boxed type), which becomes `nil` in Lua when no target is set. Always check for nil!

```lua
local nav = peripheral.wrap("front")
local angle = nav.getRelativeAngle()
if angle then
    print("Heading offset:", angle)
else
    print("No navigation target set")
end
```

### Optical Sensor (`optical_sensor`)

**Source:** `OpticalSensorPeripheral.java`

| Method | Returns | Description |
|---|---|---|
| `hasHit()` | `boolean` | Whether the laser has hit a block |
| `getDistance()` | `float` | Distance to hit block (blocks) |
| `getBlock()` | `string` | Block registry name of hit block |
| `getRange()` | `float` | Current max detection range |
| `setRange(blocks)` | — | Set max detection range (takes `int`) |

### Swivel Bearing (`swivel_bearing`) — Read-Only!

**Source:** `SwivelBearingPeripheral.java`

| Method | Returns | Description |
|---|---|---|
| `getTargetAngle()` | `double` | Current target angle in **degrees** |
| `getTargetAngleRad()` | `double` | Current target angle in **radians** |

> **CRITICAL:** There is NO `setTargetAngle()` method! Angle control must be done via rotational input to the gear (Sequenced Gearshift `rotate()`).

### Torsion Spring (`torsion_spring`)

**Source:** `TorsionSpringPeripheral.java`

| Method | Returns | Description |
|---|---|---|
| `setLimit(limit)` | — | Set angle limit (takes `int`, only works when spring is static) |
| `getAngle()` | `float` | Current spring angle |
| `getAngleRad()` | `double` | Current angle in radians |
| `getLimit()` | `int` | Current angle limit |
| `isRunning()` | `boolean` | Whether the spring is currently rotating (not static) |

---

## 4. Aeronautics/Simulated - Actuator Peripherals

### Sequenced Gearshift (Create CC Peripheral)

**Source:** https://wiki.createmod.net/users/cc-tweaked-integration/sequenced-gearshift

> **CRITICAL:** `rotate(angle)` is **RELATIVE** — "Rotates connected components **by** a set angle", NOT "to a set angle"!

| Method | Returns | Description |
|---|---|---|
| `rotate(angle, [modifier])` | — | Rotate **by** `angle` degrees. `angle`: positive integer. `modifier`: integer in [-2..2], negative = reverse |
| `move(distance, [modifier])` | — | Rotate to move piston/pulley/gantry by `distance`. Same parameter rules |
| `isRunning()` | `boolean` | Whether the gearshift is currently spinning |

**Parameter Rules:**
- `angle`: Must be a **positive integer**. For backward rotation, use negative `modifier`.
- `modifier` (default: 1): Integer in range [-2..2]. Values outside range are ignored (defaults to 1).

```lua
local sg = peripheral.wrap("right")

-- Rotate 15 degrees forward at normal speed
sg.rotate(15, 1)

-- Rotate 10 degrees backward at 2x speed
sg.rotate(10, -2)

-- Delta-based control pattern (for PID):
local current_angle = 0  -- Track this!
local target_angle = 15
local delta = target_angle - current_angle
if math.abs(delta) >= 1 then
    local rot_angle = math.max(1, math.floor(math.abs(delta) + 0.5))
    local modifier = delta > 0 and 1 or -1
    sg.rotate(rot_angle, modifier)
    current_angle = target_angle  -- Update tracking
end
```

---

## 5. Create CC: Tweaked - Peripherals

**Source:** https://wiki.createmod.net/users

### Rotation Speed Controller

**Source:** https://wiki.createmod.net/users/cc-tweaked-integration/rotational-speed-controller

| Method | Returns | Description |
|---|---|---|
| `setTargetSpeed(speed)` | — | Set target rotation speed. `speed`: integer in [-256..256], values outside are clamped |
| `getTargetSpeed()` | `number` | Current target speed in RPM |

```lua
local rsc = peripheral.wrap("back")
rsc.setTargetSpeed(128)  -- 128 RPM
print(rsc.getTargetSpeed())  -- 128
```

### Speedometer

**Source:** https://wiki.createmod.net/users/cc-tweaked-integration/speedometer

| Method | Returns | Description |
|---|---|---|
| `getSpeed()` | `number` | Current rotation speed in RPM |

**Events:**
| Event | Returns | Description |
|---|---|---|
| `speed_change` | `number` | Triggers when network speed changes; returns new RPM |

```lua
local speedo = peripheral.wrap("left")
print(speedo.getSpeed())

-- Event-based monitoring
local _, newSpeed = os.pullEvent("speed_change")
print("Speed changed to:", newSpeed)
```

### Display Link

**Source:** https://wiki.createmod.net/users/cc-tweaked-integration/display-link

| Method | Returns | Description |
|---|---|---|
| `setCursorPos(x, y)` | — | Set cursor position (can be outside bounds) |
| `getCursorPos()` | `x, y` | Current cursor position |
| `getSize()` | `height, width` | Display size |
| `isColor()` / `isColour()` | `boolean` | Whether display supports color |
| `write(text)` | — | Write text to internal buffer (does NOT push to display) |
| `writeBytes(bytes)` | — | Write raw bytes to buffer |
| `clearLine()` | — | Clear current line in buffer |
| `clear()` | — | Clear entire buffer |
| `update()` | — | **Push buffer to display** (required after write operations!) |

```lua
local disp = peripheral.wrap("front")
disp.clear()
disp.setCursorPos(1, 1)
disp.write("Hello!")
disp.update()  -- REQUIRED!
```

### Creative Motor

**Source:** https://wiki.createmod.net/users/cc-tweaked-integration/creative-motor

| Method | Returns | Description |
|---|---|---|
| `setGeneratedSpeed(speed)` | — | Set generated speed. Integer in [-256..256], clamped |
| `getGeneratedSpeed()` | `number` | Current generated speed in RPM |

---

## 6. CC: Tweaked Core APIs

**Source:** https://tweaked.cc

### `peripheral` API

| Function | Returns | Description |
|---|---|---|
| `peripheral.wrap(side)` | `table` | Get peripheral object on given side |
| `peripheral.getType(side)` | `string \| nil` | Get peripheral type name on side |
| `peripheral.exists(side)` | `boolean` | Check if peripheral exists on side |
| `peripheral.getNames()` | `{string...}` | List all peripheral names (networked) |
| `peripheral.getMethod(name, method)` | `function \| nil` | Check if peripheral has method |

**Valid sides:** `"left"`, `"right"`, `"front"`, `"back"`, `"top"`, `"bottom"`

### `redstone` API

| Function | Returns | Description |
|---|---|---|
| `redstone.setOutput(side, boolean)` | — | Set redstone output. **Takes boolean, NOT 0-15!** |
| `redstone.getInput(side)` | `number` | Get redstone input strength (0-15) |
| `redstone.getAnalogOutputSide(side)` | `number` | Get analog output (0-15) |
| `redstone.getAnalogInput(side)` | `number` | Get analog input (0-15) |

> **NOTE:** For 0-15 analog output, use `redstone_relay` peripheral, not the built-in `redstone` API.

### `os` API

| Function | Returns | Description |
|---|---|---|
| `os.pullEvent([filter])` | `event, p1, p2, ...` | Wait for event (blocks/yields) |
| `os.pullEvent(0)` | `event, p1, p2, ...` or `nil` | **Peek** next event without consuming |
| `os.sleep(seconds)` | — | Pause execution (rounds up to 0.05s increments) |
| `os.clock()` | `number` | Current time in seconds since computer start |
| `os.startTimer(seconds)` | `number` | Create timer, returns timer ID |

### `parallel` API

| Function | Returns | Description |
|---|---|---|
| `parallel.waitForAny(func1, func2, ...)` | results of finished function | Run functions concurrently until ANY finishes |
| `parallel.waitForAll(func1, func2, ...)` | — | Run functions concurrently until ALL finish |

> Each parallel function gets its own event queue.

### `keys` Constants

Common key codes: `keys.up`, `keys.down`, `keys.left`, `keys.right`, `keys.enter`,
`keys.q`, `keys.a`, `keys.s`, `keys.d`, `keys.w`, `keys.e`, `keys.escape`, etc.

### Global Functions (`_G`)

| Function | Returns | Description |
|---|---|---|
| `read([replaceChar, history, completeFn, default])` | `string` | Read user input from terminal |
| `print(...)` | `number` | Print values with newline |
| `write(text)` | `number` | Write text without newline |
| `sleep(seconds)` | — | Alias of `os.sleep()` |

---

## 7. Critical Notes and Known Issues

### Sable Bug #638: Peripheral Side IDs Change on Rotation

**Issue:** When a contraption is assembled and rotated, peripheral side mappings change.
A modem on "left" might become "back" after rotation.

**Workaround:** Attach peripherals to **top/bottom** faces only. These are least affected by rotation.

### Sub-Level API Errors

All `sublevel.*` methods (except `isInPlotGrid()`) will **throw an error** if the computer
is not on an assembled Sub-Level. Always check first:

```lua
if sublevel and sublevel.isInPlotGrid and sublevel.isInPlotGrid() then
    -- Safe to use sublevel API
end
```

### Sequenced Gearshift is RELATIVE

`rotate(angle)` rotates **by** `angle` degrees from current position, NOT **to** `angle`.
You must track current angle and compute delta yourself.

### Navigation Table Returns nil

`getRelativeAngle()` returns Java `Float` (boxed), which becomes `nil` in Lua when no
navigation target is set. Always handle nil:

```lua
local angle = nav.getRelativeAngle() or 0
```

### Display Link Requires `update()`

All `write()`, `clear()`, etc. operations write to an internal buffer. You MUST call
`update()` to push changes to the actual display.

### `os.sleep()` Discards Events

`os.sleep()` internally uses timers but does NOT listen for other events. Any event
during sleep is discarded. Use `os.pullEvent(0)` for non-blocking checks instead.

### Air Pressure Affects Thrust

At higher altitudes, air pressure decreases, reducing propeller thrust. PID controllers
may need altitude compensation for consistent performance.

---

## Peripheral Registration List

All peripherals registered by Aeronautics/Simulated (from `ComputerCraftPeripherals.java`):

| Block Entity | Peripheral Type |
|---|---|
| `ALTITUDE_SENSOR` | `altitude_sensor` |
| `GIMBAL_SENSOR` | `gimbal_sensor` |
| `NAVIGATION_TABLE` | `navigation_table` |
| `OPTICAL_SENSOR` | `optical_sensor` |
| `SWIVEL_BEARING` | `swivel_bearing` |
| `VELOCITY_SENSOR` | `velocity_sensor` |
| `TORSION_SPRING` | `torsion_spring` |
| `LINKED_TYPEWRITER` | `linked_typewriter` |
| `DIRECTIONAL_LINKED_RECEIVER` | `directional_linked_receiver` |
| `MODULATING_LINKED_RECEIVER` | `modulating_linked_receiver` |
| `DOCKING_CONNECTOR` | `docking_connector` |
| `NAMEPLATE` | `nameplate` |
