package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.brakestate.BrakeStateManager;

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
		this.commandsBuilder = new PivotCommandsBuilder(this);
		this.positionRequest = pivotStuff.positionRequest();
		this.resetAngleFilter = new MedianFilter(PivotConstants.MEDIAN_FILTER_SIZE);

		motor.resetPosition(PivotConstants.MINIMUM_ACHIEVABLE_ANGLE);

		updateInputs();
		resetResetFilter();
	}

	private void resetResetFilter() {
		for (int i = 0; i < PivotConstants.MEDIAN_FILTER_SIZE; i++) {
			resetAngleFilter.calculate(pivotStuff.positionSignal().getLatestValue().getRotations());
		}
	}

	public PivotCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public void setBreak(boolean shouldBreak) {
		motor.setBrake(shouldBreak);
	}

	void setPower(double power) {
		motor.setPower(power);
	}

	void setPosition(Rotation2d position) {
		motor.applyAngleRequest(positionRequest.withSetPoint(position));
	}

	void stop() {
		motor.stop();
	}

	void stayInPlace() {
		motor.applyAngleRequest(positionRequest.withSetPoint(pivotStuff.positionSignal().getLatestValue()));
	}

	boolean isAtAngle(Rotation2d targetAngle, Rotation2d tolerance) {
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
