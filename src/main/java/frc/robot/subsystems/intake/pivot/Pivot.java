package frc.robot.subsystems.intake.pivot;

import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.request.cansparkmax.SparkMaxAngleRequest;
import frc.utils.GBSubsystem;

public class Pivot extends GBSubsystem {

	private final PivotStuff pivotStuff;
	private final ControllableMotor motor;
	private final AbsoluteEncoder encoder;
	private final PivotCommandBuilder commandBuilder;
	private final IRequest<Rotation2d> positionRequest;

	public Pivot(PivotStuff pivotStuff) {
		super(pivotStuff.logPath());
		this.pivotStuff = pivotStuff;
		this.encoder = pivotStuff.absoluteEncoder();
		this.motor = pivotStuff.motor();
		this.commandBuilder = new PivotCommandBuilder();
		this.positionRequest = pivotStuff.positionRequest();
	}

	public PivotCommandBuilder getCommandBuilder() {
		return commandBuilder;
	}

	public Rotation2d getPosition() {
		return pivotStuff.positionSignal().getLatestValue();
	}

	public void setPower(double power) {
		motor.setPower(power);
	}

	public void updateSignals() {
		motor.updateSignals(pivotStuff.positionSignal(), pivotStuff.voltageSignal());
	}

	@Override
	protected void subsystemPeriodic() {
		updateSignals();
	}

}
