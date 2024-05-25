package frc.utils.calibration.staticcharacterization;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.GBSubsystem;
import frc.utils.roborioutils.RoborioUtils;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

class FindKg extends Command {

    private final Consumer<Double> voltageConsumer;

    private final DoubleSupplier velocitySupplier;

    private final DoubleSupplier stillVoltage;

    private final Timer timer;

    private final String subsystemName;

    private double currentVoltage;

    private double lastVoltage;

    private double cycleCounter;

    public FindKg(GBSubsystem subsystem, DoubleSupplier stillVoltage, Consumer<Double> voltageConsumer,
            DoubleSupplier velocitySupplier) {
        this.timer = new Timer();
        this.voltageConsumer = voltageConsumer;
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
            currentVoltage -= RoborioUtils.getAverageRoborioCycleTime() * StaticCharacterizationConstants.RAMP_VOLTS_PER_SEC;
            voltageConsumer.accept(currentVoltage);
        }
    }

    @Override
    public boolean isFinished() {
        return cycleCounter > StaticCharacterizationConstants.CYCLES_STABLE_NUMBER || lastVoltage <= 0;
    }

    @Override
    public void end(boolean interrupted) {
        voltageConsumer.accept(lastVoltage);
        lastVoltage = Math.max(lastVoltage, 0);
        timer.stop();
        String toLog = (interrupted ? "got interrupted" : "finished") + ", ";
        Logger.recordOutput(StaticCharacterizationConstants.LOG_PATH + "KG OF " + subsystemName, toLog + lastVoltage);
    }

}
