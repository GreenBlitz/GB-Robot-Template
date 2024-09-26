package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.signal.InputSignal;
import frc.utils.GBSubsystem;

public class Pivot extends GBSubsystem {

	private final ControllableMotor motor;
	private final InputSignal[] inputSignals;
	private final IRequest<Rotation2d> positionRequest;
	private final InputSignal<Rotation2d> positionSignal;
	private final PivotCommandsBuilder pivotCommandsBuilder;
	private final MedianFilter resetFilter;

	public Pivot(
		String logPath,
		ControllableMotor motor,
		IRequest<Rotation2d> positionRequest,
		InputSignal<Rotation2d> positionSignal,
		InputSignal... inputSignals
	) {
		super(logPath);
		this.motor = motor;
		this.positionSignal = positionSignal;
		this.positionRequest = positionRequest;
		this.inputSignals = inputSignals;
		this.pivotCommandsBuilder = new PivotCommandsBuilder(this);
		this.resetFilter = new MedianFilter(PivotConstants.MEDIAN_FILTER_SIZE);

		motor.resetPosition(PivotConstants.MINIMUM_ACHIEVABLE_ANGLE);
		updateSignals();
	}

	public PivotCommandsBuilder getPivotCommandsBuilder() {
		return pivotCommandsBuilder;
	}

	private void updateSignals() {
		motor.updateSignals(positionSignal);
		motor.updateSignals(inputSignals);
	}

	@Override
	public void subsystemPeriodic() {
		updateSignals();
		if (PivotConstants.MINIMUM_ACHIEVABLE_ANGLE.getRotations() > resetFilter.calculate(positionSignal.getLatestValue().getRotations())) {
			motor.resetPosition(PivotConstants.MINIMUM_ACHIEVABLE_ANGLE);
		}
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	public void stop() {
		motor.stop();
	}

	public void setPower(double power) {
		motor.setPower(power);
	}

	public void setTargetPosition(Rotation2d targetPosition) {
		motor.applyAngleRequest(positionRequest.withSetPoint(targetPosition));
	}

	public boolean isAtPosition(Rotation2d targetPosition, Rotation2d angleTolerance) {
		return MathUtil.isNear(targetPosition.getRotations(), positionSignal.getLatestValue().getRotations(), angleTolerance.getRotations());
	}

}
