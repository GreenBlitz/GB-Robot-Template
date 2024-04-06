package frc.utils.calibration.autocalibration.kpfinding;

import edu.wpi.first.math.Pair;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;

public class FindMaxPBeforeOscillate extends PFinding {

    private final double multiPFactor;

    protected FindMaxPBeforeOscillate(
            boolean isSetControlNeedToRunPeriodic,
            double wantedAccuracyPercent, double timeoutForActionSeconds, double multiPFactor,
            Pair<Double, Double> valuesToRunFor, Pair<Double, Double> accuracyRangeBestToWorst,
            DoubleSupplier currentValueSupplier, DoubleSupplier currentKpValueSupplier,
            Consumer<Double> setControl, Consumer<Double> setKp,
            Predicate<Double> isAtPose,
            Runnable stopAtEnd
    ) {
        super(
                isSetControlNeedToRunPeriodic,
                wantedAccuracyPercent, timeoutForActionSeconds,
                valuesToRunFor, accuracyRangeBestToWorst,
                currentValueSupplier, currentKpValueSupplier,
                setControl, setKp,
                isAtPose,
                stopAtEnd
        );

        this.multiPFactor = multiPFactor;
    }


    @Override
    public void initialize() {
        setIsInitTrue();
        wasSmall = false;
        wasBig = false;
        accuracyPercent = wantedAccuracyPercent;
    }

    @Override
    public void execute() {
        if (isInit) {
            TIMER.restart();

            wasSmall = false;
            wasBig = false;

            double currentPosition = currentValueSupplier.getAsDouble();

            replaceTargetValue();

            edgeValue = currentPosition;
            setControl.accept(usedTargetValue);

            setIsExecuteTrue();
        }
        else if (isExe) {
            if (isSetControlNeedToRunPeriodic) {
                setControl.accept(usedTargetValue);
            }

            double currentPosition = currentValueSupplier.getAsDouble();

            if (currentPosition <= usedTargetValue) {
                wasSmall = true;
            } else {
                wasBig = true;
            }

            if (wasSmall ^ wasBig) {
                edgeValue = Math.abs(edgeValue - usedTargetValue) > Math.abs(currentPosition - usedTargetValue) ? currentPosition : edgeValue;
            } else {
                edgeValue = Math.abs(edgeValue - usedTargetValue) < Math.abs(currentPosition - usedTargetValue) ? currentPosition : edgeValue;
            }

            if (isAtPose.test(usedTargetValue) || TIMER.hasElapsed(timeoutForActionSeconds)) {
                setIsEndTrue();
            }
        }
        else if (isEnd) {
            TIMER.stop();

            double error = Math.abs(edgeValue - usedTargetValue);
            accuracyPercent = getAccuracyPercent(error);

            if (accuracyPercent >= wantedAccuracyPercent) {
                setKp.accept(currentKpValueSupplier.getAsDouble() * multiPFactor);
                setIsInitTrue();
            } else {
                setKp.accept(currentKpValueSupplier.getAsDouble() / multiPFactor);
            }
        }
    }


    @Override
    public boolean isFinished() {
        return accuracyPercent < wantedAccuracyPercent;
    }

}
