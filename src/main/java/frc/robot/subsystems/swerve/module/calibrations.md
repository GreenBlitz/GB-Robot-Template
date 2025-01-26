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
- [ ] Motor ID
- [ ] Inverted
- [ ] Neutral Mode
- [ ] Current Limit
- [ ] Gear Ratio (should use RotorToSensorRatio)
- [ ] Encoder Usage and ID (should use fuse)
- [ ] FF (ks, kv, ka)
- [ ] PID
- [ ] Use ContinuousWrap
- [ ] Control mode (motion magic, voltage, torque)
- [ ] Enable/Disable FOC (only talonfx)

SteerSimulation:
-----------------------
- [ ] Moment of inertia
- [ ] DCMotor

Drive:
-----------------------
- [ ] Motor ID
- [ ] Inverted
- [ ] Neutral Mode
- [ ] Current Limit
- [ ] Gear Ratio (should use RotorToSensorRatio)
- [ ] FF (ks, kv, ka)
- [ ] PID
- [ ] Control mode (motion magic, voltage, torque)
- [ ] Enable/Disable FOC (only talonfx)

DriveSimulation:
-----------------------
 - [ ] Moment of inertia
 - [ ] DCMotor
