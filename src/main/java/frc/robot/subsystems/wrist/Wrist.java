package frc.robot.subsystems.wrist;

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

	public void stop() {
		motor.stop();
	}

	public void setPower(double power) {
		motor.setPower(power);
	}

	public void setTargetPosition(Rotation2d position) {
		motor.applyAngleRequest(positionRequest.withSetPoint(position));
	}

}
