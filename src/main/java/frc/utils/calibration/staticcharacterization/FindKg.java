package frc.utils.calibration.staticcharacterization;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

class FindKg extends Command {

    private final Consumer<Double> voltageConsumer;

    private final BooleanSupplier isMoving;

    private final Timer timer = new Timer();

    private final double stillVoltage;

    private double currentVoltage;

    private double lastVoltage;

    public FindKg(GBSubsystem subsystem, double stillVoltage, Consumer<Double> voltageConsumer, BooleanSupplier isMoving) {
        this.voltageConsumer = voltageConsumer;
        this.isMoving = isMoving;
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
        currentVoltage -= timer.get() * StaticCharacterizationConstants.RAMP_VOLTS_PER_SEC;
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
        String toLog = interrupted ? "got interrupted" : "finished";
        Logger.recordOutput(
                "Calibration/static/KG OF " + getRequirements().toArray()[0].getClass().getSimpleName(),
                toLog + lastVoltage
        );
    }

}
