package frc.robot.subsystems.lifter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.Conversions;
import org.littletonrobotics.junction.Logger;

public class Lifter extends GBSubsystem {

	private final ControllableMotor motor;
	private final LifterCommandBuilder lifterCommandsBuilder;
	private final InputSignal<Rotation2d> positionSignal;
	private final IDigitalInput limitSwitch;
	private final DigitalInputInputsAutoLogged limitSwitchInputs;
	private final double drumRadius;

	public Lifter(String logPath, ControllableMotor motor, InputSignal<Rotation2d> positionSignal, IDigitalInput limitSwitch, double drumRadius) {
		super(logPath);

		this.motor = motor;
		this.lifterCommandsBuilder = new LifterCommandBuilder(this);
		this.positionSignal = positionSignal;

		this.limitSwitch = limitSwitch;
		this.limitSwitchInputs = new DigitalInputInputsAutoLogged();

		this.drumRadius = drumRadius;

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
		return expectedPositionMeters < convertToMeters(positionSignal.getLatestValue());
	}

	protected boolean isLower(double expectedPositionMeters) {
		return !isHigher(expectedPositionMeters);
	}

	public LifterCommandBuilder getCommandsBuilder() {
		return lifterCommandsBuilder;
	}

	public boolean isLimitSwitchPressed() {
		return limitSwitchInputs.debouncedValue;
	}

	@Override
	protected void subsystemPeriodic() {
		if (LifterConstants.MINIMUM_ACHIEVABLE_POSITION_METERS > convertToMeters(positionSignal.getLatestValue())) {
			motor.resetPosition(convertFromMeters(LifterConstants.MINIMUM_ACHIEVABLE_POSITION_METERS));
		}

		updateInputs();
	}

	private void updateInputs() {
		motor.updateInputs(positionSignal);

		limitSwitch.updateInputs(limitSwitchInputs);

		Logger.recordOutput(getLogPath() + "limit switch pressed", limitSwitchInputs.debouncedValue);
		Logger.recordOutput(getLogPath() + "lifter position in meters", convertToMeters(positionSignal.getLatestValue()));
	}

	private double convertToMeters(Rotation2d motorPosition) {
		return Conversions.angleToDistance(motorPosition, drumRadius);
	}

	private Rotation2d convertFromMeters(double mechanismPosition) {
		return Conversions.distanceToAngle(mechanismPosition, drumRadius);
	}

}
