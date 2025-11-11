package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.math.ToleranceMath;

public class FlyWheel extends GBSubsystem {

	private final ControllableMotor rightMotor;
	private final ControllableMotor leftMotor;

	private final IRequest<Rotation2d> velocityRequest;
	private final IRequest<Double> voltageRequest;

	private final InputSignal<Double> voltageSignalRightMotor;
	private final InputSignal<Double> voltageSignalLeftMotor;
	private final InputSignal<Rotation2d> velocitySignalRightMotor;
	private final InputSignal<Rotation2d> velocitySignalLeftMotor;

	public FlyWheel(
		String logPath,
		IRequest<Rotation2d> velocityRequest,
		IRequest<Double> voltageRequest,
		InputSignal<Rotation2d> velocitySignalRightMotor,
		InputSignal<Double> voltageSignalRightMotor,
		InputSignal<Rotation2d> velocitySignalLeftMotor,
		InputSignal<Double> voltageSignalLeftMotor,
		ControllableMotor rightMotor,
		ControllableMotor lettMotor
	) {
		super(logPath);
		this.velocityRequest = velocityRequest;
		this.voltageRequest = voltageRequest;
		this.velocitySignalRightMotor = velocitySignalRightMotor;
		this.voltageSignalRightMotor = voltageSignalRightMotor;
		this.velocitySignalLeftMotor = velocitySignalLeftMotor;
		this.voltageSignalLeftMotor = voltageSignalLeftMotor;
		this.rightMotor = rightMotor;
		this.leftMotor = lettMotor;
	}

	public void setVoltage(double voltage) {
		leftMotor.applyRequest(voltageRequest.withSetPoint(voltage));
		rightMotor.applyRequest(voltageRequest.withSetPoint(voltage));
	}

	public void setVelocity(Rotation2d velocity) {
		leftMotor.applyRequest(velocityRequest.withSetPoint(velocity));
		rightMotor.applyRequest(velocityRequest.withSetPoint(velocity));
	}

	public Rotation2d getVelocityRightMotor() {
		return velocitySignalRightMotor.getLatestValue();
	}

	public Rotation2d getVelocityLeftMotor() {
		return velocitySignalLeftMotor.getLatestValue();
	}

	public boolean isLeftAtVelocity(Rotation2d chekVelocity) {
		return ToleranceMath.isNear(getVelocityLeftMotor().getRotations(), chekVelocity.getRotations(), Constants.VELOCITY_TOLERANCE);
	}

	public boolean isRightAtVelocity(Rotation2d chekVelocity) {
		return ToleranceMath.isNear(getVelocityRightMotor().getRotations(), chekVelocity.getRotations(), Constants.VELOCITY_TOLERANCE);
	}

	public void stop() {
		leftMotor.stop();
		rightMotor.stop();
	}

	public void setBrake(boolean brake) {
		leftMotor.setBrake(brake);
		rightMotor.setBrake(brake);
	}

	@Override
	protected void subsystemPeriodic() {
		rightMotor.updateInputs(velocitySignalRightMotor, voltageSignalRightMotor);
		leftMotor.updateInputs(voltageSignalLeftMotor, velocitySignalLeftMotor);
	}

}
