package frc.utils.calibration.autocalibration.kpfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;
import frc.utils.commands.GBCommand;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;

public class FindMaxPBeforeOscillate extends GBCommand {

    private final boolean isSetControlNeedToRunPeriodic;
    private final double wantedAccuracyPercent, timeoutForActionSeconds, multiPFactor;
    private final Pair<Double, Double> valuesToRunFor, accuracyRangeBestToWorst;
    private final DoubleSupplier currentValueSupplier, currentKpValueSupplier;
    private final Consumer<Double> setControl, setKp;
    private final Predicate<Double> isAtPose;
    private final Runnable stopAtEnd;


    private final Timer TIMER;
    private boolean isCheckingMin;
    private boolean isInit, isExe, isEnd;
    private double edgeValue;
    private double accuracyPercent, usedTargetValue;


    private boolean hasEnded = false;


    protected FindMaxPBeforeOscillate(
            boolean isSetControlNeedToRunPeriodic,
            double wantedAccuracyPercent, double timeoutForActionSeconds, double multiPFactor,
            Pair<Double, Double> valuesToRunFor, Pair<Double, Double> accuracyRangeBestToWorst,
            DoubleSupplier currentValueSupplier, DoubleSupplier currentKpValueSupplier,
            Consumer<Double> setControl, Consumer<Double> setKp,
            Predicate<Double> isAtPose,
            Runnable stopAtEnd
    ) {
        this.TIMER = new Timer();

        this.isInit = true;
        this.isExe = false;
        this.isEnd = false;

        this.isSetControlNeedToRunPeriodic = isSetControlNeedToRunPeriodic;

        this.wantedAccuracyPercent = wantedAccuracyPercent;
        this.timeoutForActionSeconds = timeoutForActionSeconds;
        this.multiPFactor = multiPFactor;

        this.valuesToRunFor = valuesToRunFor;
        this.usedTargetValue = valuesToRunFor.getSecond();

        this.accuracyRangeBestToWorst = accuracyRangeBestToWorst;

        this.currentValueSupplier = currentValueSupplier;
        this.currentKpValueSupplier = currentKpValueSupplier;

        this.setControl = setControl;
        this.setKp = setKp;
        this.isAtPose = isAtPose;
        this.stopAtEnd = stopAtEnd;
    }


    private void setIsInitTrue() {
        this.isInit = true;
        isEnd = false;
        isExe = false;
    }

    private void setIsExecuteTrue() {
        this.isExe = true;
        isInit = false;
        isEnd = false;

    }

    private void setIsEndTrue() {
        this.isEnd = true;
        isInit = false;
        isExe = false;
    }

    public void replaceTargetValue() {
        usedTargetValue = (usedTargetValue == valuesToRunFor.getFirst()) ? valuesToRunFor.getSecond() : valuesToRunFor.getFirst();
    }

    @Override
    public void initialize() {
        if (!hasEnded) {
            setIsInitTrue();
            accuracyPercent = wantedAccuracyPercent;
        }
    }

    @Override
    public void execute() {
        if (!hasEnded) {
            System.out.println("high/ current kp -> " + currentKpValueSupplier.getAsDouble());
            if (isInit) {
                TIMER.restart();

                double currentPosition = currentValueSupplier.getAsDouble();

                replaceTargetValue();
                isCheckingMin = (currentPosition > valuesToRunFor.getFirst() && currentPosition > valuesToRunFor.getSecond())
                        || (currentPosition > valuesToRunFor.getFirst() && currentPosition > valuesToRunFor.getSecond());
                edgeValue = currentPosition;
                setControl.accept(usedTargetValue);

                setIsExecuteTrue();
            } else if (isExe) {
                if (isSetControlNeedToRunPeriodic) {
                    setControl.accept(usedTargetValue);
                }

                double currentPosition = currentValueSupplier.getAsDouble();

                if (isCheckingMin) {
                    if (edgeValue > currentPosition) {
                        edgeValue = currentPosition;
                    }
                } else {
                    if (edgeValue < currentPosition) {
                        edgeValue = currentPosition;
                    }
                }

                if (isAtPose.test(usedTargetValue) || TIMER.hasElapsed(timeoutForActionSeconds)) {
                    setIsEndTrue();
                }
            } else if (isEnd) {
                TIMER.stop();

                double error = Math.abs(edgeValue - usedTargetValue);
                System.out.println("high/ edge -> " + edgeValue);
                System.out.println("high/ target -> " + usedTargetValue);
                System.out.println("high/ error -> " + error);
                accuracyPercent = 100 - (100 / (accuracyRangeBestToWorst.getSecond() - accuracyRangeBestToWorst.getFirst() + 1)) * error;

                if (accuracyPercent >= wantedAccuracyPercent) {
                    setKp.accept(currentKpValueSupplier.getAsDouble() * multiPFactor);
                    setIsInitTrue();
                } else {
                    setKp.accept(currentKpValueSupplier.getAsDouble() / multiPFactor);
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        System.out.println("high/ accuracy -> " + accuracyPercent);
        System.out.println("high/ accuracy wanted -> " + wantedAccuracyPercent);
        System.out.println("high/ hasEnded -> " + hasEnded);
        return accuracyPercent < wantedAccuracyPercent || hasEnded;
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            hasEnded = true;
        }
        stopAtEnd.run();
    }

}
