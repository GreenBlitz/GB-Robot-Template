package frc.utils.calibration.staticcharacterization;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.GBSubsystem;
import frc.utils.cycletimeutils.CycleTimeUtils;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

class FindKg extends Command {

    private final Consumer<Double> voltageConsumer;

    private final Consumer<Double> setKgMinusKs;

    private final DoubleSupplier velocitySupplier;

    private final DoubleSupplier stillVoltage;

    private final Timer timer;

    private final String subsystemName;

    private double currentVoltage;

    private double lastVoltage;

    private double cycleCounter;

    public FindKg(GBSubsystem subsystem, DoubleSupplier stillVoltage, Consumer<Double> voltageConsumer,
            DoubleSupplier velocitySupplier, Consumer<Double> setKgMinusKs) {
        this.timer = new Timer();
        this.voltageConsumer = voltageConsumer;
        this.setKgMinusKs = setKgMinusKs;
        this.velocitySupplier = velocitySupplier;
        this.stillVoltage = stillVoltage;
        this.subsystemName = subsystem.getName();
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        currentVoltage = stillVoltage.getAsDouble();
        cycleCounter = 0;
        timer.restart();
    }

    @Override
    public void execute() {
        if (velocitySupplier.getAsDouble() < -StaticCharacterizationConstants.VELOCITY_DEADBAND) {
            cycleCounter++;
        }
        else {
            cycleCounter = 0;

            lastVoltage = currentVoltage;
            currentVoltage -= CycleTimeUtils.getCurrentCycleTime() * StaticCharacterizationConstants.RAMP_VOLTS_PER_SEC;
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
        lastVoltage = Math.max(lastVoltage, 0);
        timer.stop();
        setKgMinusKs.accept(lastVoltage);
        String toLog = (interrupted ? "got interrupted" : "finished") + ", ";
        Logger.recordOutput(StaticCharacterizationConstants.LOG_PATH + "KG MINUS KS OF " + subsystemName, toLog + lastVoltage);
    }

}
