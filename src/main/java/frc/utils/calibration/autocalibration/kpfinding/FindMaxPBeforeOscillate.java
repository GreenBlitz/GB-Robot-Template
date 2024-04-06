package frc.utils.calibration.autocalibration.kpfinding;

import edu.wpi.first.math.Pair;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;

public class FindMaxPBeforeOscillate extends PFinding {

    private final double multiPFactor;

    private boolean isErrorNeedToBeCheck;

    protected FindMaxPBeforeOscillate(
            boolean isSetControlNeedToRunPeriodic,
            double tolerance, double timeoutForActionSeconds, double multiPFactor,
            Pair<Double, Double> valuesToRunFor,
            DoubleSupplier currentValueSupplier, DoubleSupplier currentKpValueSupplier,
            Consumer<Double> setControl, Consumer<Double> setKp,
            Predicate<Double> isAtPose,
            Runnable stopAtEnd
    ) {
        super(
                isSetControlNeedToRunPeriodic,
                tolerance, timeoutForActionSeconds,
                valuesToRunFor,
                currentValueSupplier, currentKpValueSupplier,
                setControl, setKp,
                isAtPose,
                stopAtEnd
        );

        this.multiPFactor = multiPFactor;
    }


    @Override
    public void execute() {
        if (isInit) {
            initFunction();
            isErrorNeedToBeCheck = false;
        }

        else if (isExe) {
            setControlPeriodic();

            final double currentValue = currentValueSupplier.getAsDouble();
            error = hasOscillated(currentValue) ? Math.max(error, Math.abs(currentValue - usedTargetValue)) : Math.min(error, Math.abs(currentValue - usedTargetValue));

            if (isNeedToBeEnd(currentValue)) {
                setIsEndTrue();
            }
        }

        else if (isEnd) {
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
        return isErrorNeedToBeCheck && error > tolerance ;
    }

}
