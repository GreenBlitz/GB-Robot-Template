package frc.utils.calibration.autocalibration.findkp;

import edu.wpi.first.math.Pair;
import frc.utils.GBSubsystem;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;

public class FindHighestKp extends KpFindingUtil {

    private final double multiPFactor;

    private boolean isErrorNeedToBeCheck;

    protected FindHighestKp(
            GBSubsystem subsystem,
            boolean isSetControlNeedToRunPeriodic,
            double tolerance,
            double timeoutForActionSeconds,
            double multiPFactor,
            Pair<Double, Double> valuesToRunFor,
            DoubleSupplier currentValueSupplier,
            DoubleSupplier currentKpValueSupplier,
            Consumer<Double> setControl,
            Consumer<Double> setKp,
            Predicate<Double> isAtPose,
            Runnable doOnEnd
    ) {
        super(
                subsystem,
                isSetControlNeedToRunPeriodic,
                tolerance,
                timeoutForActionSeconds,
                valuesToRunFor,
                currentValueSupplier,
                currentKpValueSupplier,
                setControl,
                setKp,
                isAtPose,
                doOnEnd
        );

        this.multiPFactor = multiPFactor;
    }

    @Override
    public void execute() {
        if (currentCommandPart.isInitialize()) {
            initFunction();
            isErrorNeedToBeCheck = false;
        }

        else if (currentCommandPart.isExecute()) {
            setControlPeriodic();

            final double currentValue = currentValueSupplier.getAsDouble();
            error = hasOscillated(currentValue) ?
                    Math.max(error, Math.abs(currentValue - usedTargetValue)) :
                    Math.min(error, Math.abs(currentValue - usedTargetValue));

            if (isNeedToBeEnd(currentValue)) {
                setIsEndTrue();
            }
        }

        else if (currentCommandPart.isEnd()) {
            TIMER.stop();

            if (error <= tolerance) {
                setIsInitTrue();
                setKp.accept(currentKpValueSupplier.getAsDouble() * multiPFactor);
            }
            else {
                isErrorNeedToBeCheck = true;
                setKp.accept(currentKpValueSupplier.getAsDouble() / multiPFactor);
            }
        }
    }

    @Override
    public boolean isFinished() {
        return isErrorNeedToBeCheck && error > tolerance;
    }

}