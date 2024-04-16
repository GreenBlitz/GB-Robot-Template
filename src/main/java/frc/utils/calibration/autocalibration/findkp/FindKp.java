package frc.utils.calibration.autocalibration.findkp;

import edu.wpi.first.math.Pair;
import frc.utils.GBSubsystem;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;

public class FindKp extends KpFindingUtil {

    private final double errorToKpValueFactor;

    protected FindKp(
            GBSubsystem subsystem,
            boolean isSetControlNeedToRunPeriodic,
            double tolerance,
            double timeoutForActionSeconds,
            double errorToKpValueFactor,
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

        this.errorToKpValueFactor = errorToKpValueFactor;
    }

    @Override
    public void execute() {
        if (currentCommandPart.isInitialize()) {
            initFunction();
        }

        else if (currentCommandPart.isExecute()) {
            setControlPeriodic();

            final double currentPosition = currentValueSupplier.getAsDouble();
            error = hasOscillated(currentPosition) ?
                    Math.max(error, Math.abs(currentPosition - usedTargetValue)) :
                    Math.min(error, Math.abs(currentPosition - usedTargetValue));

            if (isNeedToBeEnd(currentPosition)) {
                setIsEndTrue();
            }
        }

        else if (currentCommandPart.isEnd()) {
            TIMER.stop();

            if (error > tolerance) {
                final double sign = hasOscillated(currentValueSupplier.getAsDouble()) ? -1 : 1;
                setKp.accept(currentKpValueSupplier.getAsDouble() + (sign * error * errorToKpValueFactor));
                setIsInitTrue();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return error <= tolerance;
    }

}