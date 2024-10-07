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
		this.commandBuilder = new PivotCommandBuilder(this);
		this.positionRequest = pivotStuff.positionRequest();

		updateSignals();
	}

	public PivotCommandBuilder getCommandBuilder() {
		return commandBuilder;
	}

	public void setBreak(boolean shouldBreak) {
		motor.setBrake(shouldBreak);
	}

	public void setPower(double power) {
		motor.setPower(power);
	}

	public void setPosition(Rotation2d position) {
		motor.applyAngleRequest(positionRequest.withSetPoint(position));
	}

	public void stop() {
		motor.stop();
	}

	public void stayInPlace() {
		motor.applyAngleRequest(positionRequest.withSetPoint(pivotStuff.positionSignal().getLatestValue()));
	}

	public boolean isAtAngle(Rotation2d targetAngle) {
		return Math.abs(pivotStuff.positionSignal().getLatestValue().getRadians() - (targetAngle).getRadians())
			< PivotConstants.POSITION_TOLERANCE.getRadians();
	}

	public void updateSignals() {
		motor.updateSignals(pivotStuff.positionSignal(), pivotStuff.voltageSignal());
	}

	@Override
	protected void subsystemPeriodic() {
		updateSignals();
	}

}
