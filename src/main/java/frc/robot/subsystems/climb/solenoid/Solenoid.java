package frc.robot.subsystems.climb.solenoid;

import frc.joysticks.SmartJoystick;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.interfaces.IMotor;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Solenoid extends GBSubsystem {

	private final IMotor motor;
	private final InputSignal<Double> voltageSignal;
	private final InputSignal<Double> powerSignal;
	private final SolenoidCommandsBuilder commandsBuilder;

	private final IDigitalInput limitSwitch;
	private final DigitalInputInputsAutoLogged limitSwitchInputs;

	public Solenoid(
		String logPath,
		IMotor motor,
		InputSignal<Double> voltageSignal,
		InputSignal<Double> powerSignal,
		IDigitalInput limitSwitch
	) {
		super(logPath);
		this.motor = motor;
		this.voltageSignal = voltageSignal;
		this.powerSignal = powerSignal;
		this.commandsBuilder = new SolenoidCommandsBuilder(this);

		this.limitSwitch = limitSwitch;
		this.limitSwitchInputs = new DigitalInputInputsAutoLogged();

		updateInputs();
	}

	public SolenoidCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public void updateInputs() {
		motor.updateSimulation();
		motor.updateInputs(voltageSignal, powerSignal);

		limitSwitch.updateInputs(limitSwitchInputs);
		Logger.recordOutput(getLogPath() + "/isAtLimitSwitch", isAtLimitSwitch());
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

	public boolean isAtLimitSwitch() {
		return limitSwitchInputs.debouncedValue;
	}

	public void applyCalibrationBindings(SmartJoystick joystick) {
		SolenoidStateHandler stateHandler = new SolenoidStateHandler(this);

		joystick.POV_UP.onTrue(stateHandler.setState(SolenoidState.INITIAL_FREE));
		joystick.POV_LEFT.onTrue(stateHandler.setState(SolenoidState.HOLD_FREE));
		joystick.POV_DOWN.onTrue(stateHandler.setState(SolenoidState.LOCKED));
	}

}
