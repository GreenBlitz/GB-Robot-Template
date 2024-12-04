package frc.robot.subsystems.motorSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.alerts.Alert;

public class MotorSubsystem extends GBSubsystem {

	private final ControllableMotor motor;
	private final MotorCommandBuilder commandBuilder;
	private IRequest<Rotation2d> velocityRequest;
	private InputSignal<Rotation2d> velocitySignal;
	private IRequest lastChangedRequest;
	private boolean wasRequestApllied;

	public MotorSubsystem(ControllableMotor motor, String logPath) {
		super(logPath);
		this.motor = motor;
		this.commandBuilder = new MotorCommandBuilder(this);
		wasRequestApllied = true;
	}

	public MotorCommandBuilder getCommandBuilder() {
		return commandBuilder;
	}

	public MotorSubsystem withVelocityControl(IRequest<Rotation2d> velocityRequest, InputSignal<Rotation2d> velocitySignal) {
		this.velocityRequest = velocityRequest;
		this.velocitySignal = velocitySignal;
		return this;
	}

	public boolean isAtVelocity(Rotation2d targetVelocity, Rotation2d tolerance) {
		return MathUtil.isNear(targetVelocity.getRotations(), getVelocityRotation2dPerSecond().getRotations(), tolerance.getRotations());
	}

	public Rotation2d getVelocityRotation2dPerSecond() {
		if (velocitySignal == null) {
			new Alert(Alert.AlertType.ERROR, "The velocity signal is null. try using '.withVelocityControl'");
		}
		return velocitySignal.getLatestValue();
	}

	public void setTargetVelocityRotation2dPerSecond(Rotation2d targetVelocity) {
		if (velocityRequest == null) {
			new Alert(Alert.AlertType.ERROR, "The request signal is null. try using '.withVelocityControl'");
		}
		velocityRequest.withSetPoint(targetVelocity);
		lastChangedRequest = velocityRequest;
		wasRequestApllied = true;
	}

	public void updateInputs() {
		motor.updateInputs(velocitySignal);
	}

	private void reapplyRequest() {
		if (!motor.isConnected()) {
			wasRequestApllied = false;
		}
		if (!wasRequestApllied && motor.isConnected()) {
			motor.applyRequest(lastChangedRequest);
			wasRequestApllied = true;
		}
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		reapplyRequest();
	}

}
