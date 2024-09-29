package frc.robot.subsystems.solenoid;

import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;

public record SolenoidComponents(String logPath, IMotor solenoid, InputSignal<Double> voltageSignal) {}
