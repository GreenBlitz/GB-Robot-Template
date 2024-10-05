package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.subsystems.GBSubsystem;

public class Wrist extends GBSubsystem {

	private final ControllableMotor motor;
	private final IRequest<Rotation2d> positionRequest;
	private final InputSignal<Rotation2d> positionSignal;
	private final WristStuff wristStuff;
	private final WristCommandsBuilder commandsBuilder;

	public Wrist(WristStuff wristStuff) {
		super(wristStuff.logPath());

		this.motor = wristStuff.motor();
		this.positionRequest = wristStuff.positionRequest();
		this.positionSignal = wristStuff.positionSignal();
		this.wristStuff = wristStuff;
		this.commandsBuilder = new WristCommandsBuilder(this);

		motor.resetPosition(new Rotation2d());

		updateInputs();
	}

	public WristCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	public void resetPosition(Rotation2d position) {
		motor.resetPosition(position);
	}

	private void updateInputs() {
		motor.updateSignals(positionSignal);
		motor.updateSignals(wristStuff.otherSignals());
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

	protected void stop() {
		motor.stop();
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void setTargetPosition(Rotation2d position) {
		motor.applyAngleRequest(positionRequest.withSetPoint(position));
	}

	protected void stayInPlace() {
		motor.applyAngleRequest(positionRequest.withSetPoint(positionSignal.getLatestValue()));
	}

	public boolean isAtPosition(Rotation2d targetPosition, Rotation2d tolerance) {
		return MathUtil.isNear(targetPosition.getRadians(), positionSignal.getLatestValue().getRadians(), tolerance.getRadians());
	}

}
