package frc.robot.subsystems.climb.lifter;

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
	private final LifterCommandsBuilder lifterCommandsBuilder;
	private final InputSignal<Rotation2d> positionSignal;
	private final IDigitalInput limitSwitch;
	private final DigitalInputInputsAutoLogged limitSwitchInputs;
	private final double drumRadius;

	public Lifter(
		String logPath,
		ControllableMotor motor,
		InputSignal<Rotation2d> positionSignal,
		IDigitalInput limitSwitch,
		double drumRadius
	) {
		super(logPath);

		this.motor = motor;
		this.lifterCommandsBuilder = new LifterCommandsBuilder(this);
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
		return positionSignal.isGreater(convertLifterPositionFromMeters(expectedPositionMeters));
	}

	protected boolean isLower(double expectedPositionMeters) {
		return !isHigher(expectedPositionMeters);
	}

	public LifterCommandsBuilder getCommandsBuilder() {
		return lifterCommandsBuilder;
	}

	public boolean isAtLimitSwitch() {
		return limitSwitchInputs.debouncedValue;
	}

	public double getPositionMeters() {
		return convertLifterPositionToMeters(positionSignal.getLatestValue());
	}

	@Override
	protected void subsystemPeriodic() {
		if (LifterConstants.MINIMUM_ACHIEVABLE_POSITION_METERS > convertLifterPositionToMeters(positionSignal.getLatestValue())) {
			motor.resetPosition(convertLifterPositionFromMeters(LifterConstants.MINIMUM_ACHIEVABLE_POSITION_METERS));
		}

		updateInputs();
		motor.updateSimulation();
	}

	private void updateInputs() {
		motor.updateInputs(positionSignal);
		limitSwitch.updateInputs(limitSwitchInputs);

		Logger.processInputs(getLogPath() + "/LimitSwitch", limitSwitchInputs);
		log();
	}

	private void log() {
		Logger.recordOutput(getLogPath() + "/isAtLimitSwitch", isAtLimitSwitch());
		Logger.recordOutput(getLogPath() + "/positionMeters", getPositionMeters());
	}

	public double convertLifterPositionToMeters(Rotation2d motorPosition) {
		return Conversions.angleToDistance(motorPosition, drumRadius);
	}

	public Rotation2d convertLifterPositionFromMeters(double mechanismPosition) {
		return Conversions.distanceToAngle(mechanismPosition, drumRadius);
	}

}
