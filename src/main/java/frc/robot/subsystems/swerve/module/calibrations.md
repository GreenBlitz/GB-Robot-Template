Modules:
-----------------------
- [x] Enter modules in the order specified in ModuleUtils.ModulePosition

ModuleConstants: (real and simulation)
-----------------------
- [x] Wheel Diameter
- [x] Coupling Ratio
- [x] Velocity At 12 Volts
- [x] Modules locations (in meters)

Encoder:
----------------------
- [x] Encoder ID
- [x] Sensor Range (should be PlusMinusHalf)
- [x] Sensor Direction (should be CounterClockwise in default sds module)

Steer:
-----------------------
- [x] Motor ID
- [x] Inverted
- [x] Neutral Mode
- [x] Current Limit
- [x] Gear Ratio (should use RotorToSensorRatio)
- [x] Encoder Usage and ID (should use fuse)
- [x] FF (ks, kv, ka)
- [x] PID
- [x] Use ContinuousWrap
- [x] Control mode (motion magic, voltage, torque)
- [x] Enable/Disable FOC (only talonfx)

SteerSimulation:
-----------------------
- [x] Moment of inertia
- [x] DCMotor

Drive:
-----------------------
- [x] Inverted
- [x] Neutral Mode
- [x] Current Limit
- [x] Gear Ratio (should use RotorToSensorRatio)
- [x] FF (ks, kv, ka)
- [x] PID
- [x] Control mode (motion magic, voltage, torque)
- [x] Enable/Disable FOC (only talonfx)

DriveSimulation:
-----------------------
 - [x] Moment of inertia
 - [x] DCMotor