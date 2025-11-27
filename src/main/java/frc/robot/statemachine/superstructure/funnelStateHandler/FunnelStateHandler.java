package frc.robot.statemachine.superstructure.funnelStateHandler;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.subsystems.roller.Roller;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;


public class FunnelStateHandler {

	private final Roller omni;
	private final Roller belly;
	
	private final IDigitalInput sensor;
	private final DigitalInputInputsAutoLogged sensorInputsAutoLogged = new DigitalInputInputsAutoLogged();

	private final String logPath;
	
	private final LoggedNetworkNumber bellyCalibrationPower = new LoggedNetworkNumber("BellyPower", 0);
	private final LoggedNetworkNumber omniCalibrationPower = new LoggedNetworkNumber("OmniPower", 0);
	
	protected FunnelState currentState = FunnelState.STOP;

	public FunnelStateHandler(Roller omni, Roller belly, String logPath, IDigitalInput sensor) {
		this.omni = omni;
		this.belly = belly;
		this.sensor = sensor;
		this.logPath = logPath + "/FunnelStateHandler";
		Logger.recordOutput(logPath, "STOP");
		sensor.updateInputs(sensorInputsAutoLogged);
	}

	public Command setState(FunnelState state) {
		Command command = switch (state) {
			case DRIVE -> drive();
			case SHOOT -> shoot();
			case INTAKE -> intake();
			case STOP -> stop();
			case CALIBRATION -> calibration();
		};
		return new ParallelCommandGroup(
			new InstantCommand(() -> Logger.recordOutput(logPath, state.name())),
			new InstantCommand(() -> currentState = state),
			command
		);
	}

	public boolean isBallAtSensor() {
		return sensorInputsAutoLogged.debouncedValue; // sensor wont work until we will put the periodic in periodic
	}

	private Command drive() {
		return new ParallelDeadlineGroup(
			belly.getCommandsBuilder().rollRotationsAtVoltageForwards(1, FunnelState.DRIVE.getBellyVoltage()).until(this::isBallAtSensor),
			omni.getCommandsBuilder().stop()
		);
	}

	private Command shoot() {
		return new ParallelCommandGroup(
			omni.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getOmniVoltage()),
			belly.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getBellyVoltage())
		);
	}

	private Command intake() {
		return new ParallelCommandGroup(
			omni.getCommandsBuilder().stop(),
			belly.getCommandsBuilder().setVoltage(FunnelState.INTAKE.getBellyVoltage())
		);
	}

	private Command stop() {
		return new ParallelCommandGroup(omni.getCommandsBuilder().stop(), belly.getCommandsBuilder().stop());
	}

	private Command calibration() {
		return new ParallelCommandGroup(
			omni.getCommandsBuilder().setPower(omniCalibrationPower::get),
			belly.getCommandsBuilder().setPower(bellyCalibrationPower::get)
		);
	}

	public void periodic() {
		sensor.updateInputs(sensorInputsAutoLogged);
	}

}
