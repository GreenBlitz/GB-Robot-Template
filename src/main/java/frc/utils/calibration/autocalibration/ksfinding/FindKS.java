package frc.utils.calibration.autocalibration.ksfinding;

import edu.wpi.first.wpilibj.Timer;
import frc.utils.GBSubsystem;
import frc.utils.commands.GBCommand;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class FindKS extends GBCommand {

    private final Timer TIMER;
    private final BooleanSupplier isMoving;
    private final Consumer<Double> setVoltageControl;
    private final double kG;

    private double lastTimeSeconds;
    private double lastVoltage;

    public FindKS(GBSubsystem subsystem, double kG, BooleanSupplier isMoving, Consumer<Double> setVoltageControl) {
        this.TIMER = new Timer();
        this.isMoving = isMoving;
        this.setVoltageControl = setVoltageControl;
        this.kG = kG;

        lastTimeSeconds = 0;
        lastVoltage = 0;

        require(subsystem);
    }

    @Override
    public void initialize() {
        TIMER.restart();
        lastTimeSeconds = TIMER.get();
    }

    @Override
    public void execute() {
        if (TIMER.get() - lastTimeSeconds >= FindKSConstants.TIME_BETWEEN_VOLTAGE_RAMPS_SECONDS) {
            lastVoltage += FindKSConstants.VOLTAGE_STEP;
            setVoltageControl.accept(lastVoltage + kG);
            lastTimeSeconds = TIMER.get();
        }
    }

    @Override
    public boolean isFinished() {
        return isMoving.getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        TIMER.stop();
        Logger.recordOutput("KS OF SYSTEM", lastVoltage);
    }
}
