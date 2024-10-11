package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.subsystems.GBSubsystem;

public class Pivot extends GBSubsystem {

	private final PivotStuff pivotStuff;
	private final ControllableMotor motor;
	private final PivotCommandsBuilder commandsBuilder;
	private final IRequest<Rotation2d> positionRequest;
	private final MedianFilter resetAngleFilter;

	public Pivot(PivotStuff pivotStuff) {
		super(pivotStuff.logPath());
		this.pivotStuff = pivotStuff;
		this.motor = pivotStuff.motor();
		this.positionRequest = pivotStuff.positionRequest();
		this.resetAngleFilter = new MedianFilter(PivotConstants.MEDIAN_FILTER_SIZE);
		this.commandsBuilder = new PivotCommandsBuilder(this);

		motor.resetPosition(PivotConstants.MINIMUM_ACHIEVABLE_ANGLE);
		updateInputs();
		updateResetFilter();
	}

	private void updateResetFilter() {
		for (int i = 0; i < PivotConstants.MEDIAN_FILTER_SIZE; i++) {
			resetAngleFilter.calculate(pivotStuff.positionSignal().getLatestValue().getRotations());
		}
	}

	public PivotCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public void setBrake(boolean shouldBreak) {
		motor.setBrake(shouldBreak);
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void setPosition(Rotation2d position) {
		motor.applyAngleRequest(positionRequest.withSetPoint(position));
	}

	protected void stop() {
		motor.stop();
	}

	protected void stayInPlace() {
		motor.applyAngleRequest(positionRequest.withSetPoint(pivotStuff.positionSignal().getLatestValue()));
	}

	public boolean isAtAngle(Rotation2d targetAngle, Rotation2d tolerance) {
		return MathUtil.isNear(targetAngle.getDegrees(), pivotStuff.positionSignal().getLatestValue().getDegrees(), tolerance.getDegrees());
	}

	private void updateInputs() {
		motor.updateSignals(pivotStuff.positionSignal(), pivotStuff.voltageSignal());
	}

	@Override
	protected void subsystemPeriodic() {
		if (
			PivotConstants.MINIMUM_ACHIEVABLE_ANGLE.getRotations()
				> resetAngleFilter.calculate(pivotStuff.positionSignal().getLatestValue().getRotations())
		) {
			motor.resetPosition(PivotConstants.MINIMUM_ACHIEVABLE_ANGLE);
		}
		updateInputs();
	}

}
