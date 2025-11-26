package frc.robot.statemachine.superstructure.funnelStateHandler;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.subsystems.roller.Roller;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.Set;

public class FunnelStateHandler {

    private final Roller omni;
    private final Roller belly;
    private final String logPath;
    private final IDigitalInput sensor;
    private final DigitalInputInputsAutoLogged sensorInputsAutoLogged = new DigitalInputInputsAutoLogged();
    private final LoggedNetworkNumber bellyCalibrationPower = new LoggedNetworkNumber("BellyPower", 0);
    private final LoggedNetworkNumber omniCalibrationPower = new LoggedNetworkNumber("OmniPower", 0);

    public FunnelStateHandler(Roller omni, Roller belly, String logPath, IDigitalInput sensor) {
        this.omni = omni;
        this.belly = belly;
        this.sensor = sensor;
        this.logPath = logPath + "/FunnelStateHandler";
        sensor.updateInputs(sensorInputsAutoLogged);
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

    public boolean isBallAtSensor() {
        return sensorInputsAutoLogged.debouncedValue;
    }

    private Command drive() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> Logger.recordOutput(logPath, "DRIVE")),
                omni.getCommandsBuilder().stop(),
                belly.getCommandsBuilder().rollRotationsAtVoltageForwards(1, FunnelState.DRIVE.getBellyVoltage()).until(this::isBallAtSensor)
        );
    }

    private Command shoot() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> Logger.recordOutput(logPath, "SHOOT")),
                omni.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getOmniVoltage()),
                belly.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getBellyVoltage())
        );
    }

    private Command intake() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> Logger.recordOutput(logPath, "INTAKE")),
                omni.getCommandsBuilder().stop(),
                belly.getCommandsBuilder().setVoltage(FunnelState.INTAKE.getBellyVoltage())
        );
    }

    private Command stop() {
        return new ParallelCommandGroup(
                new InstantCommand(() -> Logger.recordOutput(logPath, "DRIVE")),
                omni.getCommandsBuilder().stop(), belly.getCommandsBuilder().stop());
    }

    private Command calibration() {
        return new DeferredCommand(() -> new ParallelCommandGroup(
                new InstantCommand(() -> Logger.recordOutput(logPath, "CALIBRATION")),
                omni.getCommandsBuilder().setPower(omniCalibrationPower.get()),
                belly.getCommandsBuilder().setPower(bellyCalibrationPower.get())
        ), Set.of(omni,belly));
    }

    public void periodic(){
        sensor.updateInputs(sensorInputsAutoLogged);
    }

}
