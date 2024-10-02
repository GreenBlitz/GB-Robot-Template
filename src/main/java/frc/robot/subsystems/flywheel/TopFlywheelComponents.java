package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;

public record TopFlywheelComponents(
	String logPath,
	ControllableMotor motor,
	boolean isMotorInverted,
	InputSignal<Double> motorVoltageSignal,
	SuppliedAngleSignal motorVelocitySignal,
	IRequest<Rotation2d> motorVelocityRequest

) {}
