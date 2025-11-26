package frc.robot.statemachine.superstructure.funnelStateHandler;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.subsystems.roller.Roller;
import org.littletonrobotics.junction.Logger;

public class FunnelStateHandler {

	private final Roller omni;
	private final Roller belly;
	private final String logPath;
	private DigitalInputInputsAutoLogged sensor;

	public FunnelStateHandler(Roller omni, Roller belly, String logPath, DigitalInputInputsAutoLogged sensor) {
		this.omni = omni;
		this.belly = belly;
		this.logPath = logPath + "/State";
	}

	public Command setState(FunnelState state) {
		return switch (state) {
			case DRIVE -> drive();
			case SHOOT -> shoot();
			case INTAKE -> intake();
			case STAY_IN_PLACE -> stayInPlace();
		};
	}

    public boolean isBallAtSensor(){
        return sensor.debouncedValue;
    }

	private Command drive() {
		Logger.recordOutput(logPath,"DRIVE");
		return new ParallelCommandGroup(
			omni.getCommandsBuilder().stop(),
			belly.getCommandsBuilder().rollRotationsAtVoltageForwards(1, FunnelConstants.DRIVE_BELLY_VOLTAGE).until(this::isBallAtSensor)
		);
	}

	private Command shoot() {
		Logger.recordOutput(logPath,"SHOOT");
		return new ParallelCommandGroup(
			omni.getCommandsBuilder().setVoltage(FunnelConstants.SHOOT_OMNI_VOLTAGE),
			belly.getCommandsBuilder().setVoltage(FunnelConstants.SHOOT_BELLY_VOLTAGE)
		);
	}

	private Command intake() {
		Logger.recordOutput(logPath,"INTAKE");
		return new ParallelCommandGroup(
			omni.getCommandsBuilder().stop(),
			belly.getCommandsBuilder().setPower(FunnelConstants.INTAKE_BELLY_VOLTAGE)
		);
	}

	private Command stayInPlace() {
		Logger.recordOutput(logPath,"STAY_IN_PLACE");
		return new ParallelCommandGroup(omni.getCommandsBuilder().stop(), belly.getCommandsBuilder().stop());
	}

}
