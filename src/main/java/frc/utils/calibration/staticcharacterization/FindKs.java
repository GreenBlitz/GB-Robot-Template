package frc.utils.calibration.staticcharacterization;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

class FindKs extends Command {

    private final Consumer<Double> voltageConsumer;

    private final Consumer<Double> updateKs;

    private final BooleanSupplier isMoving;

    private final Timer timer = new Timer();

    private double currentVoltage;

    private double lastVoltage;

    public FindKs(GBSubsystem subsystem, Consumer<Double> voltageConsumer, BooleanSupplier isMoving,
            Consumer<Double> updateKs) {
        this.voltageConsumer = voltageConsumer;
        this.isMoving = isMoving;
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
        return isMoving.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        voltageConsumer.accept(lastVoltage);
        timer.stop();

        updateKs.accept(lastVoltage);
        String toLog = interrupted ? "got interrupted" : "finished";
        Logger.recordOutput(
                "Calibration/static/KS OF " + getRequirements().toArray()[0].getClass().getSimpleName(),
                toLog + lastVoltage
        );
    }

}