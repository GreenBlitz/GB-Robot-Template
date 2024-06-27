package frc.utils.calibration.sysid;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class SysIdCalibrator {

    private final SysIdRoutine sysIdRoutine;

    private final GBSubsystem usedSubSystem;

    private final boolean isCTRE;

    //todo - check if comment is right

    /**
     * IMPORTANT:
     *
     * @param voltageSetControl - note that this function needs to use kg in it so the mechanism won't move because of gravity.
     */
    public SysIdCalibrator(boolean isCTRE, GBSubsystem subsystem, Consumer<Double> voltageSetControl, double voltageStepVolts,
            double rampRateVoltsPerSecond) {
        this(
                isCTRE,
                subsystem,
                voltageSetControl,
                voltageStepVolts,
                rampRateVoltsPerSecond,
                SysIdConstants.DEFAULT_TIMEOUT_SECONDS
        );
    }

    /**
     * IMPORTANT:
     *
     * @param voltageSetControl - note that this function needs to use kg in it so the mechanism won't move because of gravity.
     */
    public SysIdCalibrator(boolean isCTRE, GBSubsystem subsystem, Consumer<Double> voltageSetControl, double voltageStep) {
        this(
                isCTRE,
                subsystem,
                voltageSetControl,
                voltageStep,
                SysIdConstants.DEFAULT_RAMP_RATE_VOLTS_PER_SECOND,
                SysIdConstants.DEFAULT_TIMEOUT_SECONDS
        );
    }

    /**
     * IMPORTANT:
     *
     * @param voltageSetControl - note that this function needs to use kg in it so the mechanism won't move because of gravity.
     */
    public SysIdCalibrator(boolean isCTRE, GBSubsystem subsystem, Consumer<Double> voltageSetControl) {
        this(
                isCTRE,
                subsystem,
                voltageSetControl,
                SysIdConstants.DEFAULT_VOLTAGE_STEP,
                SysIdConstants.DEFAULT_RAMP_RATE_VOLTS_PER_SECOND,
                SysIdConstants.DEFAULT_TIMEOUT_SECONDS
        );
    }

    /**
     * IMPORTANT:
     *
     * @param voltageSetControl - note that this function needs to use kg in it so the mechanism won't move because of gravity.
     */
    public SysIdCalibrator(boolean isCTRE, GBSubsystem subsystem, Consumer<Double> voltageSetControl, double voltageStepVolts,
            double rampRateVoltsPerSecond, double timeoutSeconds) {
        this.usedSubSystem = subsystem;
        this.isCTRE = isCTRE;

        final SysIdRoutine.Config config = new SysIdRoutine.Config(
                Volts.of(rampRateVoltsPerSecond).per(Seconds.of(1)),
                Volts.of(voltageStepVolts),
                Seconds.of(timeoutSeconds),
                this.isCTRE
                        ? (state) -> SignalLogger.writeString("state", state.toString())
                        : (state) -> Logger.recordOutput("state", state.toString())
        );
        final SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> voltageSetControl.accept(volts.in(Volts)),
                null,
                usedSubSystem,
                usedSubSystem.getName()
        );

        this.sysIdRoutine = new SysIdRoutine(config, mechanism);
    }

    public Command getSysIdCommand(boolean isQuasistatic, SysIdRoutine.Direction direction) {
        return isQuasistatic ? getSysIdQuasistatic(direction) : getSysIdDynamic(direction);
    }

    public Command getSysIdQuasistatic(SysIdRoutine.Direction direction) {
        Command command = sysIdRoutine.quasistatic(direction);
        return getAppropriateCommand(command);
    }

    public Command getSysIdDynamic(SysIdRoutine.Direction direction) {
        Command command = sysIdRoutine.dynamic(direction);
        return getAppropriateCommand(command);
    }

    private Command getAppropriateCommand(Command sysIdCommand) {
        sysIdCommand.addRequirements(usedSubSystem);
        return isCTRE ? getCTRECommand(sysIdCommand) : sysIdCommand;
    }

    private Command getCTRECommand(Command sysIdCommand) {
        return new SequentialCommandGroup(new InstantCommand(SignalLogger::start), sysIdCommand);
    }

}
