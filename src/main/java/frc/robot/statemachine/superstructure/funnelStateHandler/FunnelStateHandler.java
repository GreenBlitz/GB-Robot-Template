package frc.robot.statemachine.superstructure.funnelStateHandler;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.subsystems.roller.Roller;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class FunnelStateHandler {

	private final Roller omni;
	private final Roller belly;
	private final String logPath;
	private DigitalInputInputsAutoLogged sensor;

	public FunnelStateHandler(Roller omni, Roller belly, String logPath, DigitalInputInputsAutoLogged sensor) {
		this.omni = omni;
		this.belly = belly;
		this.logPath = logPath + "/FunnelState";
	}

	public Command setState(FunnelState state) {
		return switch (state) {
			case DRIVE -> drive();
			case SHOOT -> shoot();
			case INTAKE -> intake();
			case STOP -> stop();
            case CALIBRATION -> calibration();
		};
	}

    public boolean isBallAtSensor(){
        return sensor.debouncedValue;
    }

	private Command drive() {
		Logger.recordOutput(logPath,"DRIVE");
		return new ParallelCommandGroup(
			omni.getCommandsBuilder().stop(),
			belly.getCommandsBuilder().rollRotationsAtVoltageForwards(
            1, FunnelState.DRIVE.getBellyVoltage()).until(this::isBallAtSensor)
		);
	}

	private Command shoot() {
		Logger.recordOutput(logPath,"SHOOT");
		return new ParallelCommandGroup(
			omni.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getOmniVoltage()),
			belly.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getBellyVoltage())
		);
	}

	private Command intake() {
		Logger.recordOutput(logPath,"INTAKE");
		return new ParallelCommandGroup(
			omni.getCommandsBuilder().stop(),
			belly.getCommandsBuilder().setVoltage(FunnelState.INTAKE.getBellyVoltage())
		);
	}

	private Command stop() {
		Logger.recordOutput(logPath,"STOP");
		return new ParallelCommandGroup(
                omni.getCommandsBuilder().stop(),
                belly.getCommandsBuilder().stop()
        );
	}

    private Command calibration() {
		Logger.recordOutput(logPath,"CALIBRATION");
		return new ParallelCommandGroup(
                omni.getCommandsBuilder().setPower(LoggedNetworkNumber),
                belly.getCommandsBuilder().setPower(LoggedNetworkNumber));
	}

}
