package frc.robot.subsystems.algaeIntake.rollers;

import frc.joysticks.SmartJoystick;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;

public class Rollers extends GBSubsystem {

	private final ControllableMotor rollers;
	private final InputSignal<Double> voltageSignal;
	private final IDigitalInput algaeSensor;
	private final DigitalInputInputsAutoLogged algaeSensorInputs;
	private final RollersCommandsBuilder commandsBuilder;

	public Rollers(String logPath, ControllableMotor rollers, InputSignal<Double> voltageSignal, IDigitalInput algaeSensor) {
		super(logPath);
		this.rollers = rollers;
		this.voltageSignal = voltageSignal;
		this.algaeSensor = algaeSensor;
		this.algaeSensorInputs = new DigitalInputInputsAutoLogged();
		this.commandsBuilder = new RollersCommandsBuilder(this);
		setDefaultCommand(commandsBuilder.stop());

		periodic();
	}

	public RollersCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public double getVoltage() {
		return voltageSignal.getLatestValue();
	}

	public boolean isAlgaeIn() {
		return algaeSensorInputs.debouncedValue;
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

	private void updateInputs() {
		rollers.updateSimulation();
		rollers.updateInputs(voltageSignal);
		algaeSensor.updateInputs(algaeSensorInputs);
	}

	public void setBrake(boolean brake) {
		rollers.setBrake(brake);
	}

	protected void setPower(double power) {
		rollers.setPower(power);
	}

	protected void stop() {
		setPower(0);
	}


	public void applyCalibrationBindings(SmartJoystick joystick) {
		joystick.A.onTrue(commandsBuilder.setPower(RollersState.IDLE.getPower()));
		joystick.B.onTrue(commandsBuilder.setPower(RollersState.INTAKE.getPower()));
		joystick.X.onTrue(commandsBuilder.setPower(RollersState.OUTTAKE.getPower()));
		joystick.Y.onTrue(commandsBuilder.stop());
	}

}
