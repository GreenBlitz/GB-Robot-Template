package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class Flywheel extends GBSubsystem {

	private final ControllableMotor rightMotor;
	private final ControllableMotor leftMotor;
	private final IRequest<Rotation2d> rightFlywheelVelocityRequest;
	private final IRequest<Rotation2d> leftFlywheelVelocityRequest;
	private final FlywheelStuff flywheelStuff;

	private final FlywheelCommandsBuilder commandsBuilder;
	private final SysIdCalibrator rightSysidCalibrator;
	private final SysIdCalibrator leftSysidCalibrator;

	public Flywheel(FlywheelStuff flywheelStuff) {
		super(flywheelStuff.logPath());
		this.rightMotor = flywheelStuff.rightFlywheel();
		this.leftMotor = flywheelStuff.leftFlywheel();
		this.rightFlywheelVelocityRequest = flywheelStuff.rightFlywheelVelocityRequest();
		this.leftFlywheelVelocityRequest = flywheelStuff.leftFlywheelVelocityRequest();
		this.flywheelStuff = flywheelStuff;
		this.commandsBuilder = new FlywheelCommandsBuilder(this);
		this.rightSysidCalibrator = new SysIdCalibrator(rightMotor.getSysidConfigInfo(), this, voltage -> setVoltages(voltage, 0));
		this.leftSysidCalibrator = new SysIdCalibrator(leftMotor.getSysidConfigInfo(), this, voltage -> setVoltages(0, voltage));

		updateInputs();
	}

	public FlywheelCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public SysIdCalibrator getRightSysidCalibrator() {
		return rightSysidCalibrator;
	}

	public SysIdCalibrator getLeftSysidCalibrator() {
		return leftSysidCalibrator;
	}

	private void updateInputs() {
		rightMotor.updateSignals(flywheelStuff.rightSignals());
		rightMotor.updateSignals(flywheelStuff.rightVelocitySignal());
		leftMotor.updateSignals(flywheelStuff.leftSignals());
		leftMotor.updateSignals(flywheelStuff.leftVelocitySignal());
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

	protected void stop() {
		rightMotor.stop();
		leftMotor.stop();
	}

	protected void setPowers(double rightPower, double leftPower) {
		rightMotor.setPower(rightPower);
		leftMotor.setPower(leftPower);
	}

	protected void setVoltages(double rightVoltage, double leftVoltage) {
		rightMotor.applyDoubleRequest(flywheelStuff.rightVoltageRequest().withSetPoint(rightVoltage));
		leftMotor.applyDoubleRequest(flywheelStuff.leftVoltageRequest().withSetPoint(leftVoltage));
	}

	protected void setTargetVelocities(Rotation2d rightFlywheelVelocity, Rotation2d leftFlywheelVelocity) {
		rightMotor.applyAngleRequest(rightFlywheelVelocityRequest.withSetPoint(rightFlywheelVelocity));
		leftMotor.applyAngleRequest(leftFlywheelVelocityRequest.withSetPoint(leftFlywheelVelocity));
	}

	//@formatter:off
	public boolean isAtVelocities(Rotation2d rightFlywheelTargetVelocity, Rotation2d leftFlywheelTargetVelocity, Rotation2d velocityPerSecondTolerance) {
		return isAtVelocities(rightFlywheelTargetVelocity, leftFlywheelTargetVelocity, velocityPerSecondTolerance, velocityPerSecondTolerance);
	}
	//@formatter:on

	public boolean isAtVelocities(
		Rotation2d rightFlywheelTargetVelocity,
		Rotation2d leftFlywheelTargetVelocity,
		Rotation2d rightVelocityPerSecondTolerance,
		Rotation2d leftVelocityPerSecondTolerance
	) {
		boolean rightFlyWheelAtVelocity = MathUtil.isNear(
			rightFlywheelTargetVelocity.getRotations(),
			flywheelStuff.rightVelocitySignal().getLatestValue().getRotations(),
			rightVelocityPerSecondTolerance.getRotations()
		);
		boolean leftFlyWheelAtVelocity = MathUtil.isNear(
			leftFlywheelTargetVelocity.getRotations(),
			flywheelStuff.leftVelocitySignal().getLatestValue().getRotations(),
			leftVelocityPerSecondTolerance.getRotations()
		);
		return rightFlyWheelAtVelocity && leftFlyWheelAtVelocity;
	}

}
