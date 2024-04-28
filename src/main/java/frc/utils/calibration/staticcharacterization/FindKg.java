package frc.utils.calibration.staticcharacterization;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

class FindKg extends Command {

    private final Consumer<Double> voltageConsumer;

    private final DoubleSupplier velocitySupplier;

    private final DoubleSupplier stillVoltage;

    private final Timer timer = new Timer();

    private double currentVoltage;

    private double lastVoltage;

    public FindKg(GBSubsystem subsystem, DoubleSupplier stillVoltage, Consumer<Double> voltageConsumer,
            DoubleSupplier velocitySupplier) {
        this.voltageConsumer = voltageConsumer;
        this.velocitySupplier = velocitySupplier;
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
        //todo - with rio utils
        currentVoltage = stillVoltage.getAsDouble() - timer.get() * StaticCharacterizationConstants.RAMP_VOLTS_PER_SEC;
        voltageConsumer.accept(currentVoltage);
    }

    @Override
    public boolean isFinished() {
        return velocitySupplier.getAsDouble() < -StaticCharacterizationConstants.VELOCITY_DEADBAND;
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
