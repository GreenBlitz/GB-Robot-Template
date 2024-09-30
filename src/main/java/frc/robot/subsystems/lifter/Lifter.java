package frc.robot.subsystems.lifter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.utils.Conversions;
import frc.utils.GBSubsystem;

public class Lifter extends GBSubsystem {

	private final ControllableMotor motor;
	private final LifterStuff lifterStuff;
	private final LifterCommandsBuilder lifterCommandsBuilder;

	public Lifter(LifterStuff lifterStuff) {
		super(lifterStuff.logPath());
		this.motor = lifterStuff.motor();
		this.lifterStuff = lifterStuff;
		this.lifterCommandsBuilder = new LifterCommandsBuilder(this);
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

	public boolean isAfter(double expectedPosition) {
		return expectedPosition > convertToMeters(lifterStuff.positionSignal().getLatestValue());
	}

	public boolean isBefore(double expectedPosition) {
		return !isAfter(expectedPosition);
	}

	public LifterStuff getLifterStuff() {
		return lifterStuff;
	}

	public LifterCommandsBuilder getLifterCommandsBuilder() {
		return lifterCommandsBuilder;
	}

	@Override
	protected void subsystemPeriodic() {
		motor.updateSignals(lifterStuff.positionSignal());
		motor.updateSignals(lifterStuff.otherSignals());
	}

	public double convertToMeters(Rotation2d motorPosition) {
		return Conversions.angleToDistance(motorPosition, lifterStuff.drumRadius());
	}

}
