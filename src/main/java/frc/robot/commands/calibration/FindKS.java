package frc.robot.commands.calibration;

import edu.wpi.first.wpilibj.Timer;
import frc.utils.commands.GBCommand;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class FindKS extends GBCommand {

    private static final double VOLTAGE_RAMP = 0.01;
    private static final double TIME_BETWEEN_VOLTAGE_RAMPS_SECONDS = 0.1;

    private final Timer TIMER;
    private final double kG;
    private final BooleanSupplier isMoving;
    private final Consumer<Double> setVoltageControl;

    private double lastTimeSeconds;
    private double lastVoltage;


    public FindKS(double kG, BooleanSupplier isMoving, Consumer<Double> setVoltageControl) {
        this.TIMER = new Timer();
        this.isMoving = isMoving;
        this.setVoltageControl = setVoltageControl;
        this.kG = kG;

        lastTimeSeconds = 0;
        lastVoltage = 0;
    }

    @Override
    public void initialize() {
        TIMER.restart();
        lastTimeSeconds = TIMER.get();
    }

    @Override
    public void execute() {
        if (TIMER.get() - lastTimeSeconds >= TIME_BETWEEN_VOLTAGE_RAMPS_SECONDS) {
            lastVoltage += VOLTAGE_RAMP;
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
