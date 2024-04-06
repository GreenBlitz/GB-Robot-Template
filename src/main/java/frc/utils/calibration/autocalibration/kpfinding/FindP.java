package frc.utils.calibration.autocalibration.kpfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;
import frc.utils.commands.GBCommand;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;

public class FindP extends GBCommand {

    private final boolean isSetControlNeedToRunPeriodic;
    private final double wantedAccuracyPercent, timeoutForActionSeconds, errorToKpValueFactor;
    private final Pair<Double, Double> valuesToRunFor, accuracyRangeBestToWorst;
    private final DoubleSupplier currentValueSupplier, currentKpValueSupplier;
    private final Consumer<Double> setControl, setKp;
    private final Predicate<Double> isAtPose;
    private final Runnable stopAtEnd;


    private final Timer TIMER;
    private boolean isInit, isExe, isEnd;
    private double edgeValue;
    private double accuracyPercent, usedTargetValue;

    private boolean wasSmall = false;
    private boolean wasBig = false;


    protected FindP(
            boolean isSetControlNeedToRunPeriodic,
            double wantedAccuracyPercent, double timeoutForActionSeconds, double errorToKpValueFactor,
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
        this.errorToKpValueFactor = errorToKpValueFactor;

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
        setIsInitTrue();
        wasSmall = false;
        wasBig = false;
        accuracyPercent = 0;
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
        } else if (isExe) {
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
        } else if (isEnd) {
            TIMER.stop();

            double sign = wasBig && wasSmall ? -1 : 1;
            double error = Math.abs(edgeValue - usedTargetValue);

            accuracyPercent = 100 - (100 / (accuracyRangeBestToWorst.getSecond() - accuracyRangeBestToWorst.getFirst() + 1)) * error;

            if (accuracyPercent < wantedAccuracyPercent) {
                setKp.accept(currentKpValueSupplier.getAsDouble() + (sign * error * errorToKpValueFactor));
                setIsInitTrue();
            }
        }
    }


    @Override
    public boolean isFinished() {
        return accuracyPercent >= wantedAccuracyPercent;
    }

    @Override
    public void end(boolean interrupted) {
        stopAtEnd.run();
    }
}
