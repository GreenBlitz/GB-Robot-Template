package frc.robot.subsystems.solenoid;

import frc.robot.hardware.interfaces.IMotor;
import frc.robot.hardware.interfaces.InputSignal;

public record SolenoidComponents(String logPath, IMotor solenoid, InputSignal<Double> voltageSignal) {}
