package frc.utils.calibration.staticcharacterization;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

class FindKs extends Command {

    private final Consumer<Double> voltageConsumer;

    private final Consumer<Double> updateKs;

    private final DoubleSupplier velocitySupplier;

    private final Timer timer = new Timer();

    private double currentVoltage;

    private double lastVoltage;

    public FindKs(GBSubsystem subsystem, Consumer<Double> voltageConsumer, DoubleSupplier velocitySupplier,
            Consumer<Double> updateKs) {
        this.voltageConsumer = voltageConsumer;
        this.velocitySupplier = velocitySupplier;
        this.updateKs = updateKs;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        currentVoltage = 0;
        timer.restart();
    }

    @Override
    public void execute() {
        lastVoltage = currentVoltage;
        currentVoltage = timer.get() * StaticCharacterizationConstants.RAMP_VOLTS_PER_SEC;
        voltageConsumer.accept(currentVoltage);
    }

    @Override
    public boolean isFinished() {
        return velocitySupplier.getAsDouble() >= StaticCharacterizationConstants.VELOCITY_DEADBAND;
    }

    @Override
    public void end(boolean interrupted) {
        voltageConsumer.accept(lastVoltage);
        timer.stop();

        updateKs.accept(lastVoltage);
        String toLog = (interrupted ? "got interrupted" : "finished") + ", ";
        Logger.recordOutput(
                StaticCharacterizationConstants.logPath + "KS OF " + getRequirements().toArray()[0].getClass().getSimpleName(),
                toLog + lastVoltage
        );
    }

}