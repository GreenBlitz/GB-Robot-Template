package frc.robot.subsystems.intake.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.subsystems.GBSubsystem;

public class Pivot extends GBSubsystem {

	private final PivotStuff pivotStuff;
	private final ControllableMotor motor;
	private final PivotCommandBuilder commandBuilder;
	private final IRequest<Rotation2d> positionRequest;

	public Pivot(PivotStuff pivotStuff) {
		super(pivotStuff.logPath());
		this.pivotStuff = pivotStuff;
		this.motor = pivotStuff.motor();
		this.positionRequest = pivotStuff.positionRequest();
		this.commandBuilder = new PivotCommandBuilder(this);

		updateInputs();
	}

	public PivotCommandBuilder getCommandBuilder() {
		return commandBuilder;
	}

	public void setBreak(boolean shouldBreak) {
		motor.setBrake(shouldBreak);
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void setPosition(Rotation2d position) {
		motor.applyAngleRequest(positionRequest.withSetPoint(position));
	}

	protected void stop() {
		motor.stop();
	}

	protected void stayInPlace() {
		motor.applyAngleRequest(positionRequest.withSetPoint(pivotStuff.positionSignal().getLatestValue()));
	}

	protected boolean isAtAngle(Rotation2d targetAngle, Rotation2d tolerance) {
		return Math.abs(pivotStuff.positionSignal().getLatestValue().getRadians() - (targetAngle).getRadians())
			< tolerance.getRadians();
	}

	private void updateInputs() {
		motor.updateSignals(pivotStuff.positionSignal(), pivotStuff.voltageSignal());
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

}
