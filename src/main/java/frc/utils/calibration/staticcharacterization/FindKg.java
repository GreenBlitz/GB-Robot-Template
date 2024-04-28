package frc.utils.calibration.staticcharacterization;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

class FindKg extends Command {

    private final Consumer<Double> voltageConsumer;

    private final BooleanSupplier isMoving;

    private final Timer timer = new Timer();

    private final DoubleSupplier stillVoltage;

    private double currentVoltage;

    private double lastVoltage;

    public FindKg(GBSubsystem subsystem, DoubleSupplier stillVoltage, Consumer<Double> voltageConsumer,
            BooleanSupplier isMoving) {
        this.voltageConsumer = voltageConsumer;
        this.isMoving = isMoving;
        this.stillVoltage = stillVoltage;
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
        currentVoltage = stillVoltage.getAsDouble() - timer.get() * (StaticCharacterizationConstants.RAMP_VOLTS_PER_SEC / 100);
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
        String toLog = (interrupted ? "got interrupted" : "finished") + ", ";
        Logger.recordOutput(
                StaticCharacterizationConstants.logPath + "KG OF " + getRequirements().toArray()[0].getClass().getSimpleName(),
                toLog + lastVoltage
        );
    }

}
