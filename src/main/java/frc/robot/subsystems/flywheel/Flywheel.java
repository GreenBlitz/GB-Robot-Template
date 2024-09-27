package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.utils.GBSubsystem;

public class Flywheel extends GBSubsystem {

	private final ControllableMotor rightMotor;
	private final ControllableMotor leftMotor;
	private final IRequest<Rotation2d> rightFlywheelVelocityRequest;
	private final IRequest<Rotation2d> leftFlywheelVelocityRequest;
	private final FlywheelStuff flywheelStuff;

	private final FlywheelCommandsBuilder commandsBuilder;

	public Flywheel(FlywheelStuff flywheelStuff) {
		super(flywheelStuff.logPath());
		this.rightMotor = flywheelStuff.rightFlywheel();
		this.leftMotor = flywheelStuff.leftFlywheel();
		this.rightFlywheelVelocityRequest = flywheelStuff.rightFlywheelVelocityRequest();
		this.leftFlywheelVelocityRequest = flywheelStuff.leftFlywheelVelocityRequest();
		this.flywheelStuff = flywheelStuff;

		this.commandsBuilder = new FlywheelCommandsBuilder(this);

		updateSignals();
	}

	public FlywheelCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	@Override
	public String getLogPath() {
		return FlyWheelConstants.LOG_PATH;
	}

	private void updateSignals() {
		rightMotor.updateSignals(flywheelStuff.rightSignals());
		rightMotor.updateSignals(flywheelStuff.rightVelocitySignal());
		leftMotor.updateSignals(flywheelStuff.leftSignals());
		leftMotor.updateSignals(flywheelStuff.leftVelocitySignal());
	}

	@Override
	protected void subsystemPeriodic() {
		updateSignals();
	}

	public void stop() {
		setPowers(0, 0);
	}

	public void setPowers(double rightPower, double leftPower) {
		rightMotor.setPower(rightPower);
		leftMotor.setPower(leftPower);
	}

	public void setTargetVelocities(Rotation2d rightFlywheelVelocity, Rotation2d leftFlywheelVelocity) {
		rightMotor.applyAngleRequest(rightFlywheelVelocityRequest.withSetPoint(rightFlywheelVelocity));
		leftMotor.applyAngleRequest(leftFlywheelVelocityRequest.withSetPoint(leftFlywheelVelocity));
	}

	//@formatter:off
	public boolean isAtVelocities(Rotation2d rightFlywheelExpectedVelocity, Rotation2d leftFlywheelExpectedVelocity, Rotation2d velocityTolerance) {
		return isAtVelocities(rightFlywheelExpectedVelocity, leftFlywheelExpectedVelocity, velocityTolerance, velocityTolerance);
	}
	//@formatter:on

	public boolean isAtVelocities(
		Rotation2d rightFlywheelExpectedVelocity,
		Rotation2d leftFlywheelExpectedVelocity,
		Rotation2d rightVelocityTolerance,
		Rotation2d leftVelocityTolerance
	) {
		boolean rightFlyWheelAtVelocity = MathUtil
			.isNear(rightFlywheelExpectedVelocity.getRotations(), rightVelocityTolerance.getRotations(), rightVelocityTolerance.getRotations());
		boolean leftFlyWheelAtVelocity = MathUtil
			.isNear(leftFlywheelExpectedVelocity.getRotations(), leftVelocityTolerance.getRotations(), leftVelocityTolerance.getRotations());
		return rightFlyWheelAtVelocity && leftFlyWheelAtVelocity;
	}

}
