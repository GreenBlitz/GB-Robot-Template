package frc.robot.subsystems.lifter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.Conversions;
import org.littletonrobotics.junction.Logger;

public class Lifter extends GBSubsystem {

	private final ControllableMotor motor;
	private final LifterStuff lifterStuff;
	private final LifterCommandsBuilder lifterCommandsBuilder;

	public Lifter(LifterStuff lifterStuff) {
		super(lifterStuff.logPath());
		this.motor = lifterStuff.motor();
		this.lifterStuff = lifterStuff;
		this.lifterCommandsBuilder = new LifterCommandsBuilder(this);
		motor.resetPosition(new Rotation2d());

		updateInputs();
	}

	public void setPower(double power) {
		motor.setPower(power);
	}

	public void stop() {
		motor.stop();
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	public boolean isHigher(double expectedPositionMeters) {
		return expectedPositionMeters < convertToMeters(lifterStuff.positionSignal().getLatestValue());
	}

	public boolean isLower(double expectedPositionMeters) {
		return !isHigher(expectedPositionMeters);
	}

	public LifterStuff getLifterStuff() {
		return lifterStuff;
	}

	public LifterCommandsBuilder getCommandsBuilder() {
		return lifterCommandsBuilder;
	}

	@Override
	protected void subsystemPeriodic() {
		if (LifterConstants.MINIMUM_ACHIVEABLE_POSITION_METERS > convertToMeters(lifterStuff.positionSignal().getLatestValue())) {
			motor.resetPosition(convertFromMeters(LifterConstants.MINIMUM_ACHIVEABLE_POSITION_METERS));
		}

		updateInputs();
	}

	private void updateInputs() {
		motor.updateSignals(lifterStuff.positionSignal());
		motor.updateSignals(lifterStuff.otherSignals());

		Logger.recordOutput("lifter position", convertToMeters(lifterStuff.positionSignal().getLatestValue()));
	}

	private double convertToMeters(Rotation2d motorPosition) {
		return Conversions.angleToDistance(motorPosition, lifterStuff.drumRadius());
	}

	private Rotation2d convertFromMeters(double mechanismPosition) {
		return Conversions.distanceToAngle(mechanismPosition, lifterStuff.drumRadius());
	}

}

