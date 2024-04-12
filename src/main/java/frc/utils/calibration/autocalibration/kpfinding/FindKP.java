package frc.utils.calibration.autocalibration.kpfinding;

import edu.wpi.first.math.Pair;
import frc.utils.GBSubsystem;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;

public class FindKP extends KPFindingUtil {

    private final double errorToKpValueFactor;

    protected FindKP(
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
        if (isInitialize) {
            initFunction();
        }
        else if (isExecute) {
            setControlPeriodic();

            final double currentPosition = currentValueSupplier.getAsDouble();
            error = hasOscillated(currentPosition) ?
                    Math.max(error, Math.abs(currentPosition - usedTargetValue)) :
                    Math.min(error, Math.abs(currentPosition - usedTargetValue));

            if (isNeedToBeEnd(currentPosition)) {
                setIsEndTrue();
            }
        }
        else if (isEnd) {
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
