package frc.robot.subsystems.elbow;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.utils.GBSubsystem;

public class Elbow extends GBSubsystem {

	private final ControllableMotor motor;
	private final IRequest<Rotation2d> positionRequest;
	private final IRequest<Double> voltageRequest;
	private final ElbowStuff elbowStuff;
	private final ElbowCommandsBuilder commandsBuilder;

	public Elbow(ElbowStuff elbowStuff) {
		super(elbowStuff.logPath());
		this.motor = elbowStuff.elbow();
		this.positionRequest = elbowStuff.positionRequest();
		this.voltageRequest = elbowStuff.voltageRequest();
		this.elbowStuff = elbowStuff;
		this.commandsBuilder = new ElbowCommandsBuilder(this);

		motor.resetPosition(ElbowConstants.MINIMUM_ACHIEVABLE_POSITION);
		updateSignals();
	}

	public ElbowCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	@Override
	protected void subsystemPeriodic() {
		updateSignals();
		if (ElbowConstants.MINIMUM_ACHIEVABLE_POSITION.getRotations() > elbowStuff.positionSignal().getLatestValue().getRotations()) {
			motor.resetPosition(ElbowConstants.MINIMUM_ACHIEVABLE_POSITION);
		}
	}

	private void updateSignals() {
		motor.updateSignals(elbowStuff.positionSignal(), elbowStuff.velocitySignal(), elbowStuff.currentSignal(), elbowStuff.voltageSignal());
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void setVoltage(double voltage) {
		motor.applyDoubleRequest(voltageRequest.withSetPoint(voltage));
	}

	protected void stayInPlace() {
		setTargetAngle(elbowStuff.positionSignal().getLatestValue());
	}

	protected void setTargetAngle(Rotation2d angle) {
		motor.applyAngleRequest(positionRequest.withSetPoint(angle));
	}

	public boolean isAtAngle(Rotation2d angle, Rotation2d tolerance) {
		return MathUtil.isNear(angle.getDegrees(), elbowStuff.positionSignal().getLatestValue().getDegrees(), tolerance.getDegrees());
	}

}
