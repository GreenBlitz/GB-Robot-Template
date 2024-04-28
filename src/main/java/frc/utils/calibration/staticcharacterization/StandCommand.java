package frc.utils.calibration.staticcharacterization;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

class StandCommand extends Command {

    private final Consumer<Double> voltageConsumer;

    private final Consumer<Double> updateKs;

    private final BooleanSupplier isMoving;

    private final Timer timer = new Timer();

    private final DoubleSupplier stillVoltage;

    private double currentVoltage;

    private double lastVoltage;

    public StandCommand(GBSubsystem subsystem, DoubleSupplier stillVoltage, Consumer<Double> voltageConsumer,
            BooleanSupplier isMoving, Consumer<Double> updateKs) {
        this.voltageConsumer = voltageConsumer;
        this.isMoving = isMoving;
        this.stillVoltage = stillVoltage;
        this.updateKs = updateKs;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        currentVoltage = stillVoltage.getAsDouble();
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
        return !isMoving.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        voltageConsumer.accept(lastVoltage);
        timer.stop();
        updateKs.accept(lastVoltage);
        Logger.recordOutput(StaticCharacterizationConstants.logPath + "KS");
    }

}