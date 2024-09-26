package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.utils.GBSubsystem;

public class Pivot extends GBSubsystem {

	private final ControllableMotor motor;
	private final IRequest<Rotation2d> positionRequest;
	private final PivotStuff pivotStuff;
	private final PivotCommandsBuilder pivotCommandsBuilder;
	private final MedianFilter resetFilter;

	public Pivot(String logPath, PivotStuff pivotStuff) {
		super(logPath);
		this.motor = pivotStuff.motor();
		this.positionRequest = pivotStuff.positionRequest();
		this.pivotStuff = pivotStuff;
		this.pivotCommandsBuilder = new PivotCommandsBuilder(this);
		this.resetFilter = new MedianFilter(PivotConstants.MEDIAN_FILTER_SIZE);

		motor.resetPosition(PivotConstants.MINIMUM_ACHIEVABLE_ANGLE);
		updateSignals();
	}

	public PivotCommandsBuilder getPivotCommandsBuilder() {
		return pivotCommandsBuilder;
	}

	@Override
	public String getLogPath() {
		return PivotConstants.LOG_PATH;
	}

	private void updateSignals() {
		motor.updateSignals(pivotStuff.positionSignal());
		motor.updateSignals(pivotStuff.inputSignals());
	}

	@Override
	public void subsystemPeriodic() {
		updateSignals();
		if (
			PivotConstants.MINIMUM_ACHIEVABLE_ANGLE.getRotations()
				> resetFilter.calculate(pivotStuff.positionSignal().getLatestValue().getRotations())
		) {
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

	public void stayInPlace() {
		setTargetPosition(pivotStuff.positionSignal().getLatestValue());
	}

	public boolean isAtPosition(Rotation2d targetPosition, Rotation2d angleTolerance) {
		return MathUtil
			.isNear(targetPosition.getRotations(), pivotStuff.positionSignal().getLatestValue().getRotations(), angleTolerance.getRotations());
	}

}
