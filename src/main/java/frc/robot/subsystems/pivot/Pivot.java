package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.DriverStationUtils;
import org.littletonrobotics.junction.Logger;

public class Pivot extends GBSubsystem {

	private final ControllableMotor motor;
	private final IRequest<Rotation2d> positionRequest;
	private final PivotStuff pivotStuff;
	private final PivotCommandsBuilder pivotCommandsBuilder;
	private final MedianFilter resetFilterRotations;

	public Pivot(PivotStuff pivotStuff) {
		super(pivotStuff.logPath());
		this.motor = pivotStuff.motor();
		this.positionRequest = pivotStuff.positionRequest();
		this.pivotStuff = pivotStuff;
		this.pivotCommandsBuilder = new PivotCommandsBuilder(this);
		this.resetFilterRotations = new MedianFilter(PivotConstants.MEDIAN_FILTER_SIZE);

		motor.resetPosition(PivotConstants.MINIMUM_ACHIEVABLE_ANGLE);
		updateInputs();
		resetResetFilter();
	}

	private void resetResetFilter() {
		for (int i = 0; i < PivotConstants.MEDIAN_FILTER_SIZE; i++) {
			resetFilterRotations.calculate(pivotStuff.positionSignal().getLatestValue().getRotations());
		}
	}

	public PivotCommandsBuilder getCommandsBuilder() {
		return pivotCommandsBuilder;
	}

	private void updateInputs() {
		motor.updateInputs(pivotStuff.positionSignal());
		motor.updateInputs(pivotStuff.inputSignals());
		motor.updateSimulation();
	}

	@Override
	public void subsystemPeriodic() {
		updateInputs();
		if (
			PivotConstants.MINIMUM_ACHIEVABLE_ANGLE.getRotations()
				> resetFilterRotations.calculate(pivotStuff.positionSignal().getLatestValue().getRotations())
				&& DriverStationUtils.isDisabled()
		) {
			motor.resetPosition(PivotConstants.MINIMUM_ACHIEVABLE_ANGLE);
		}
		Logger.recordOutput("PivotPose3d", getPose3d());
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	protected void stop() {
		motor.stop();
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void setTargetPosition(Rotation2d targetPosition) {
		motor.applyRequest(positionRequest.withSetPoint(targetPosition));
	}

	protected void stayInPlace() {
		setTargetPosition(pivotStuff.positionSignal().getLatestValue());
	}

	public boolean isAtPosition(Rotation2d targetPosition, Rotation2d angleTolerance) {
		return MathUtil
			.isNear(targetPosition.getRotations(), pivotStuff.positionSignal().getLatestValue().getRotations(), angleTolerance.getRotations());
	}

	public Pose3d getPose3d() {
		return new Pose3d(
			PivotConstants.ROBOT_RELATIVE_PIVOT_POSITION,
			new Rotation3d(0, -pivotStuff.positionSignal().getLatestValue().getRadians(), 0)
		);
	}

}
