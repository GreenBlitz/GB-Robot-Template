Modules:
-----------------------
- [ ] Enter modules in the order specified in ModuleUtils.ModulePosition

ModuleConstants: (real and simulation)
-----------------------
- [ ] Wheel Diameter
- [ ] Coupling Ratio
- [ ] Velocity At 12 Volts
- [ ] Modules locations (in meters)

Encoder:
----------------------
- [ ] Encoder ID
- [ ] Sensor Range (should be PlusMinusHalf)
- [ ] Sensor Direction (should be CounterClockwise in default sds module)

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
- [ ] Moment of inertia
- [ ] DCMotor

Drive:
-----------------------
- [x] Inverted
- [x] Neutral Mode
- [x] Current Limit
- [x] Gear Ratio (should use RotorToSensorRatio)
- [ ] FF (ks, kv, ka)
- [ ] PID
- [ ] Control mode (motion magic, voltage, torque)
- [x] Enable/Disable FOC (only talonfx)

DriveSimulation:
-----------------------
 - [ ] Moment of inertia
 - [ ] DCMotor