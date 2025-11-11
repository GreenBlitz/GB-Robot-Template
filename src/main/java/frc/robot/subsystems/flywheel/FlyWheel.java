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

	private final InputSignal<Double> voltageSignal;
	private final InputSignal<Rotation2d> velocitySignal;

	public FlyWheel(
		String logPath,
		IRequest<Rotation2d> velocityRequest,
		IRequest<Double> voltageRequest,
		InputSignal<Rotation2d> velocitySignal,
		InputSignal<Double> voltageSignal,
		ControllableMotor rightMotor,
		ControllableMotor lettMotor
	) {
		super(logPath);
		this.velocityRequest = velocityRequest;
		this.voltageRequest = voltageRequest;
		this.velocitySignal = velocitySignal;
		this.voltageSignal = voltageSignal;
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

	public Rotation2d getVelocity() {
		return velocitySignal.getLatestValue();
	}

	public double getVoltage() {
		return voltageSignal.getLatestValue();
	}

	public boolean isAtVelocity(Rotation2d velocity) {
		return ToleranceMath.isNear(getVelocity().getRotations(), velocity.getRotations(), Constants.VELOCITY_TOLERANCE);
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
		rightMotor.updateInputs(velocitySignal, voltageSignal);
		leftMotor.updateInputs(velocitySignal, voltageSignal);
	}

}
