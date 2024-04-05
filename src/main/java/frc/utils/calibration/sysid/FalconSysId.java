package frc.utils.calibration.sysid;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.GBSubsystem;

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class FalconSysId {

    private final SysIdRoutine sysIdRoutine;
    private final GBSubsystem usedSubSystem;


    public FalconSysId(GBSubsystem subsystem, Consumer<Double> voltageSetControl, double voltageStepVolts, double rampRateVoltsPerSecond) {
        this(subsystem, voltageSetControl, voltageStepVolts, rampRateVoltsPerSecond, SysIdConstants.DEFAULT_TIMEOUT);
    }

    public FalconSysId(GBSubsystem subsystem, Consumer<Double> voltageSetControl, double voltageStep) {
        this(subsystem, voltageSetControl, voltageStep, SysIdConstants.DEFAULT_RAMP_RATE, SysIdConstants.DEFAULT_TIMEOUT);
    }

    public FalconSysId(GBSubsystem subsystem, Consumer<Double> voltageSetControl) {
        this(subsystem, voltageSetControl, SysIdConstants.DEFAULT_VOLTAGE_STEP, SysIdConstants.DEFAULT_RAMP_RATE, SysIdConstants.DEFAULT_TIMEOUT);
    }

    public FalconSysId(GBSubsystem subsystem, Consumer<Double> voltageControl, double voltageStepVolts, double rampRateVoltsPerSecond, double timeout) {
        this.usedSubSystem = subsystem;

        SysIdRoutine.Config config = new SysIdRoutine.Config(
                Volts.of(rampRateVoltsPerSecond).per(Seconds.of(1)),
                Volts.of(voltageStepVolts),
                Seconds.of(timeout),
                (state) -> SignalLogger.writeString("state", state.toString())
        );
        SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> voltageControl.accept(volts.in(Volts)),
                null,
                usedSubSystem
        );
        this.sysIdRoutine = new SysIdRoutine(config, mechanism);
    }



    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        Command command = sysIdRoutine.quasistatic(direction);
        command.addRequirements(usedSubSystem);
        return new SequentialCommandGroup(
                new InstantCommand(SignalLogger::start),
                command,
                new InstantCommand(SignalLogger::stop)
        );
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        Command command = sysIdRoutine.dynamic(direction);
        command.addRequirements(usedSubSystem);
        return new SequentialCommandGroup(
                new InstantCommand(SignalLogger::start),
                command,
                new InstantCommand(SignalLogger::stop)
        );
    }

}
