package frc.robot.subsystems.algaeIntake.rollers;

import frc.joysticks.SmartJoystick;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Rollers extends GBSubsystem {

	private final ControllableMotor rollers;
	private final InputSignal<Double> voltageSignal;
	private final InputSignal<Double> currentSignal;
	private final InputSignal<Double> powerSignal;
	private final IDigitalInput algaeSensor;
	private final DigitalInputInputsAutoLogged algaeSensorInputs;

	private double lastCyclePower;
	private boolean isAlgaeCurrentlyIn;
	private boolean cantDetectAlgaeInByCurrent;

	private final RollersCommandsBuilder commandsBuilder;

	public Rollers(
		String logPath,
		ControllableMotor rollers,
		InputSignal<Double> voltageSignal,
		InputSignal<Double> currentSignal,
		InputSignal<Double> powerSignal,
		IDigitalInput algaeSensor
	) {
		super(logPath);
		this.rollers = rollers;
		this.voltageSignal = voltageSignal;
		this.currentSignal = currentSignal;
		this.powerSignal = powerSignal;
		this.algaeSensor = algaeSensor;
		this.algaeSensorInputs = new DigitalInputInputsAutoLogged();

		this.lastCyclePower = voltageSignal.getLatestValue();

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

	private boolean isAlgaeInByCurrent() {
		return isCurrentSpiking() && isPowerConstant() && !cantDetectAlgaeInByCurrent;
	}

	public boolean isPowerConstant() {
		return powerSignal.getLatestValue() == lastCyclePower;
	}

	public boolean isCurrentSpiking() {
		return currentSignal.isGreater(RollersConstants.IS_CURRENT_SPIKING_THRESHOLD);
	}

	public boolean isAlgaeCurrentlyIn() {
		return isAlgaeCurrentlyIn;
	}

	@Override
	protected void subsystemPeriodic() {
		boolean lastIsPowerStable = isPowerConstant();
		boolean lastIsCurrentSpiking = isCurrentSpiking();
		lastCyclePower = powerSignal.getLatestValue();
		updateInputs();
		if (isPowerConstant() && !lastIsPowerStable) {
			cantDetectAlgaeInByCurrent = true;
		}
		if (!isCurrentSpiking() && lastIsCurrentSpiking) {
			cantDetectAlgaeInByCurrent = false;
		}
		if (isAlgaeInByCurrent()) {
			isAlgaeCurrentlyIn = true;
		}
		if (powerSignal.getLatestValue() < 0) {
			isAlgaeCurrentlyIn = false;
		}
		Logger.recordOutput(getLogPath() + "/isPowerStable", isPowerConstant());
		Logger.recordOutput(getLogPath() + "/isCurrentSpiking", isCurrentSpiking());
		Logger.recordOutput(getLogPath() + "/isAlgaeIn", isAlgaeCurrentlyIn());
	}

	private void updateInputs() {
		rollers.updateSimulation();
		rollers.updateInputs(voltageSignal, currentSignal, powerSignal);
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
