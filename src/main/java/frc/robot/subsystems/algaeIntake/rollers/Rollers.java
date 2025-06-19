package frc.robot.subsystems.algaeIntake.rollers;

import frc.joysticks.SmartJoystick;
import frc.robot.hardware.YishaiDistanceSensor;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Rollers extends GBSubsystem {

	private final ControllableMotor rollers;
	private final InputSignal<Double> voltageSignal;
	private final InputSignal<Double> currentSignal;
	private final InputSignal<Double> powerSignal;
	private final YishaiDistanceSensor distanceSensor;

	private final RollersCommandsBuilder commandsBuilder;

	public Rollers(
		String logPath,
		ControllableMotor rollers,
		InputSignal<Double> voltageSignal,
		InputSignal<Double> currentSignal,
		InputSignal<Double> powerSignal,
		YishaiDistanceSensor distanceSensor
	) {
		super(logPath);
		this.rollers = rollers;
		this.voltageSignal = voltageSignal;
		this.currentSignal = currentSignal;
		this.powerSignal = powerSignal;
		this.distanceSensor = distanceSensor;

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
		return distanceSensor.getDistanceMeters() < RollersConstants.DISTANCE_FROM_SENSOR_TO_CONSIDER_ALGAE_IN_METERS;
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		Logger.recordOutput(getLogPath() + "/isAlgaeIn", isAlgaeIn());
	}

	private void updateInputs() {
		rollers.updateSimulation();
		rollers.updateInputs(voltageSignal, currentSignal, powerSignal);
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
