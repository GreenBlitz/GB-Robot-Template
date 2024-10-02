package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;

public record FlywheelComponents(
	String logPath,
	ControllableMotor motor,
	boolean isMotorInverted,
	InputSignal<Double> VoltageSignal,
	SuppliedAngleSignal VelocitySignal,
	IRequest<Rotation2d> VelocityRequest
) {}
