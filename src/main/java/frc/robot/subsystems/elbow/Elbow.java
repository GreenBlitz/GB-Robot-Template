package frc.robot.subsystems.elbow;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.subsystems.GBSubsystem;

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
		updateInputs();
	}

	public ElbowCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	@Override
	protected void subsystemPeriodic() {
		if (ElbowConstants.MINIMUM_ACHIEVABLE_POSITION.getRotations() > elbowStuff.positionSignal().getLatestValue().getRotations()) {
			motor.resetPosition(ElbowConstants.MINIMUM_ACHIEVABLE_POSITION);
		}
		updateInputs();
	}

	private void updateInputs() {
		motor.updateInputs(elbowStuff.positionSignal(), elbowStuff.velocitySignal(), elbowStuff.currentSignal(), elbowStuff.voltageSignal());
		motor.updateSimulation();
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void setVoltage(double voltage) {
		motor.applyRequest(voltageRequest.withSetPoint(voltage));
	}

	protected void stayInPlace() {
		setTargetAngle(elbowStuff.positionSignal().getLatestValue());
	}

	protected void setTargetAngle(Rotation2d angle) {
		motor.applyRequest(positionRequest.withSetPoint(angle));
	}

	public boolean isAtAngle(Rotation2d angle, Rotation2d tolerance) {
		return MathUtil.isNear(angle.getDegrees(), elbowStuff.positionSignal().getLatestValue().getDegrees(), tolerance.getDegrees());
	}

	public Pose3d getPose3D() {
		return new Pose3d(
				ElbowConstants.ELBOW_POSITION_RELATIVE_TO_ROBOT,
				new Rotation3d(0, elbowStuff.positionSignal().getLatestValue().getRadians() , 0)
		);
	}

}
