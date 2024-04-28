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

    private double counter;

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
        counter = 0;
        timer.restart();
    }

    @Override
    public void execute() {
        currentVoltage = stillVoltage.getAsDouble() - timer.get() * (StaticCharacterizationConstants.RAMP_VOLTS_PER_SEC / 100);
        voltageConsumer.accept(currentVoltage);
        if (!isMoving.getAsBoolean()) {
            counter++;
        }
        else {
            counter = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return counter > 8;
    }

    @Override
    public void end(boolean interrupted) {
        voltageConsumer.accept(currentVoltage);
        timer.stop();
        updateKs.accept(currentVoltage);
        Logger.recordOutput(StaticCharacterizationConstants.logPath + "KS", currentVoltage);
    }

}