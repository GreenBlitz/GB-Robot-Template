package frc.utils.calibration.staticcharacterization;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

class FindKg extends Command {

    private static final double RAMP_VOLTS_PER_SEC = 0.1;

    private final Consumer<Double> voltageConsumer;

    private final DoubleSupplier velocitySupplier;

    private final Timer timer = new Timer();

    private final double stillVoltage;

    private double currentVoltage;

    private double lastVoltage;

    public FindKg(GBSubsystem subsystem, double stillVoltage, Consumer<Double> voltageConsumer, DoubleSupplier velocitySupplier) {
        this.voltageConsumer = voltageConsumer;
        this.velocitySupplier = velocitySupplier;
        this.stillVoltage = stillVoltage;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        currentVoltage = stillVoltage;
        timer.restart();
    }

    @Override
    public void execute() {
        lastVoltage = currentVoltage;
        currentVoltage -= timer.get() * RAMP_VOLTS_PER_SEC;
        voltageConsumer.accept(currentVoltage);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(velocitySupplier.getAsDouble()) > 1E-4;
    }

    @Override
    public void end(boolean interrupted) {
        voltageConsumer.accept(lastVoltage);
        timer.stop();
        String toLog = interrupted ? "got interrupted" : "finished";
        Logger.recordOutput(
                "Calibration/static/KG OF " + getRequirements().toArray()[0].getClass().getSimpleName(),
                toLog + lastVoltage
        );
    }

}
