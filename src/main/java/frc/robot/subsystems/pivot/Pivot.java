package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.request.phoenix6.Phoenix6AngleRequest;
import frc.robot.hardware.signal.InputSignal;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

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
		motor.resetPosition(PivotRealConstants.BACKWARD_ANGLE_LIMIT);

		this.positionSignal = positionSignal;

		this.positionRequest = positionRequest;
		this.inputSignals = inputSignals;

		this.pivotCommandsBuilder = new PivotCommandsBuilder(this);

		this.resetFilter = new MedianFilter(PivotConstants.MEDIAN_FILTER_SIZE);
		motor.updateSignals(positionSignal);
	}

	@Override
	public void subsystemPeriodic() {
		motor.updateSignals(positionSignal);
		motor.updateSignals(inputSignals);

		if(PivotRealConstants.BACKWARD_ANGLE_LIMIT.getRotations() > resetFilter.calculate(positionSignal.getLatestValue().getRotations())){
			motor.resetPosition(PivotRealConstants.BACKWARD_ANGLE_LIMIT);
		}
	}

	public PivotCommandsBuilder getPivotCommandsBuilder(){
		return pivotCommandsBuilder;
	}

	public boolean isAtPosition(Rotation2d targetPosition, Rotation2d angleTolerance) {
		return MathUtil.isNear(targetPosition.getRotations(), positionSignal.getLatestValue().getRotations(), angleTolerance.getRotations());
	}

	public void setTargetPosition(Rotation2d targetPosition) {
		motor.applyAngleRequest(positionRequest.withSetPoint(targetPosition));
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	public void setPower(double power) {
		motor.setPower(power);
	}

	public void stop() {
		motor.stop();
	}

}
