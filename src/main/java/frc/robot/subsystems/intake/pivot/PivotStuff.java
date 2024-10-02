package frc.robot.subsystems.intake.pivot;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;

public record PivotStuff(
	String logPath,
	IMotor motor,
	InputSignal<Double> voltageSignal,
	InputSignal<Rotation2d> positionSignal,
	AbsoluteEncoder absoluteEncoder
) {}
