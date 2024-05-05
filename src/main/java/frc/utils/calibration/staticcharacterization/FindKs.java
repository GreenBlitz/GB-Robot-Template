package frc.utils.calibration.staticcharacterization;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.GBSubsystem;
import frc.utils.roborioutils.RoborioUtils;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

class FindKs extends Command {

    private final Consumer<Double> voltageConsumer;

    private final Consumer<Double> updateKs;

    private final DoubleSupplier velocitySupplier;

    private final Timer timer;

    private final String subsystemName;

    private double currentVoltage;

    private double lastVoltage;

    private double cycleCounter;

    public FindKs(GBSubsystem subsystem, Consumer<Double> voltageConsumer, DoubleSupplier velocitySupplier,
            Consumer<Double> updateKs) {
        this.timer = new Timer();
        this.voltageConsumer = voltageConsumer;
        this.velocitySupplier = velocitySupplier;
        this.updateKs = updateKs;
        this.subsystemName = subsystem.getName();
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        currentVoltage = 0;
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
            currentVoltage += RoborioUtils.getCurrentRoborioCycleTime() * StaticCharacterizationConstants.RAMP_VOLTS_PER_SEC;
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
        updateKs.accept(lastVoltage);
        String toLog = (interrupted ? "got interrupted" : "finished") + ", ";
        Logger.recordOutput(StaticCharacterizationConstants.LOG_PATH + "KS OF " + subsystemName, toLog + lastVoltage);
    }

}
