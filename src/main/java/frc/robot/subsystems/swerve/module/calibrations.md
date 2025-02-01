Modules:
-----------------------
- [ ] Enter modules in the order specified in ModuleUtils.ModulePosition

ModuleConstants: (real and simulation)
-----------------------
- [X] Wheel Diameter
- [ ] Coupling Ratio
- [ ] Velocity At 12 Volts
- [ ] Modules locations (in meters)

Encoder:
----------------------
- [X] Encoder ID
- [X] Sensor Range (should be PlusMinusHalf)
- [X] Sensor Direction (should be CounterClockwise in default sds module)

Steer:
-----------------------
- [X] Motor ID
- [X] Inverted
- [X] Neutral Mode
- [X] Current Limit
- [X] Gear Ratio (should use RotorToSensorRatio)
- [X] Encoder Usage and ID (should use fuse)
- [X] FF (ks, kv, ka)
- [X] PID
- [X] Use ContinuousWrap
- [X] Control mode (motion magic, voltage, torque)
- [X] Enable/Disable FOC (only talonfx)

SteerSimulation:
-----------------------
- [ ] Moment of inertia
- [ ] DCMotor

Drive:
-----------------------
- [X] Motor ID
- [X] Inverted
- [X] Neutral Mode
- [X] Current Limit
- [X] Gear Ratio (should use RotorToSensorRatio)
- [ ] FF (ks, kv, ka)
- [ ] PID
- [ ] Control mode (motion magic, voltage, torque)
- [ ] Enable/Disable FOC (only talonfx)

DriveSimulation:
-----------------------
 - [ ] Moment of inertia
 - [ ] DCMotor
