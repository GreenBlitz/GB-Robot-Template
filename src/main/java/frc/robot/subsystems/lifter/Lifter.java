package frc.robot.subsystems.lifter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.Conversions;
import org.littletonrobotics.junction.Logger;

public class Lifter extends GBSubsystem {

	private final ControllableMotor motor;
	private final LifterStuff lifterStuff;
	private final LifterCommandsBuilder lifterCommandsBuilder;
	private final IDigitalInput limitSwitch;
	private final DigitalInputInputsAutoLogged limitSwitchInputs;

	public Lifter(LifterStuff lifterStuff) {
		super(lifterStuff.logPath());

		this.motor = lifterStuff.motor();

		this.lifterStuff = lifterStuff;
		this.lifterCommandsBuilder = new LifterCommandsBuilder(this);

		this.limitSwitch = lifterStuff.limitSwitch();
		this.limitSwitchInputs = new DigitalInputInputsAutoLogged();

		motor.resetPosition(new Rotation2d());
		updateInputs();
	}

	protected void setPower(double power) {
		motor.setPower(power);
	}

	protected void stop() {
		motor.stop();
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	protected boolean isHigher(double expectedPositionMeters) {
		return expectedPositionMeters < convertToMeters(lifterStuff.positionSignal().getLatestValue());
	}

	protected boolean isLower(double expectedPositionMeters) {
		return !isHigher(expectedPositionMeters);
	}

	public LifterCommandsBuilder getCommandsBuilder() {
		return lifterCommandsBuilder;
	}

	public boolean isLimitSwitchPressed() {
		return limitSwitchInputs.debouncedValue;
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

		limitSwitch.updateInputs(limitSwitchInputs);

		Logger.recordOutput(getLogPath() + "limit switch pressed", limitSwitchInputs.debouncedValue);
		Logger.recordOutput(getLogPath() + "lifter position in meters", convertToMeters(lifterStuff.positionSignal().getLatestValue()));
	}

	private double convertToMeters(Rotation2d motorPosition) {
		return Conversions.angleToDistance(motorPosition, lifterStuff.drumRadius());
	}

	private Rotation2d convertFromMeters(double mechanismPosition) {
		return Conversions.distanceToAngle(mechanismPosition, lifterStuff.drumRadius());
	}

}

