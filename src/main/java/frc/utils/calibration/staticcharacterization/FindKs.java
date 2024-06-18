package frc.utils.calibration.staticcharacterization;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.GBSubsystem;
import frc.utils.cycletimeutils.CycleTimeUtils;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

class FindKs extends Command {

    private final Consumer<Double> voltageConsumer;

    private final Consumer<Double> setKgPlusKs;

    private final DoubleSupplier velocitySupplier;

    private final Timer timer;

    private final String subsystemName;

    private final double startingVoltage;

    private double currentVoltage;

    private double lastVoltage;

    private double cycleCounter;

    public FindKs(GBSubsystem subsystem, double startingVoltage, Consumer<Double> voltageConsumer, DoubleSupplier velocitySupplier,
            Consumer<Double> setKgPlusKs) {
        this.timer = new Timer();
        this.voltageConsumer = voltageConsumer;
        this.velocitySupplier = velocitySupplier;
        this.setKgPlusKs = setKgPlusKs;
        this.subsystemName = subsystem.getName();
        this.startingVoltage = startingVoltage;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        currentVoltage = startingVoltage;
        cycleCounter = 0;
        timer.restart();
    }

    @Override
    public void execute() {
        if (velocitySupplier.getAsDouble() >= StaticCharacterizationConstants.VELOCITY_DEADBAND) {
            cycleCounter++;
        }
        else {
            cycleCounter = 0;

            lastVoltage = currentVoltage;
            currentVoltage += CycleTimeUtils.getCurrentCycleTime() * StaticCharacterizationConstants.RAMP_VOLTS_PER_SEC;
            voltageConsumer.accept(currentVoltage);
        }
    }

    @Override
    public boolean isFinished() {
        return cycleCounter > StaticCharacterizationConstants.CYCLES_STABLE_NUMBER;
    }

    @Override
    public void end(boolean interrupted) {
        voltageConsumer.accept(lastVoltage);
        timer.stop();
        setKgPlusKs.accept(lastVoltage);
        String toLog = (interrupted ? "got interrupted" : "finished") + ", ";
        Logger.recordOutput(StaticCharacterizationConstants.LOG_PATH + "KG PLUS KS OF " + subsystemName, toLog + lastVoltage);
    }

}
