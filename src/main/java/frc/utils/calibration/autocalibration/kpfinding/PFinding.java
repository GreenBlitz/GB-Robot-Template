package frc.utils.calibration.autocalibration.kpfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;
import frc.utils.commands.GBCommand;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;

public abstract class PFinding extends GBCommand {

    protected final boolean isSetControlNeedToRunPeriodic;
    protected final double wantedAccuracyPercent, timeoutForActionSeconds;
    protected final Pair<Double, Double> valuesToRunFor, accuracyRangeBestToWorst;
    protected final DoubleSupplier currentValueSupplier, currentKpValueSupplier;
    protected final Consumer<Double> setControl, setKp;
    protected final Predicate<Double> isAtPose;
    protected final Runnable stopAtEnd;


    protected final Timer TIMER;
    protected boolean isInit, isExe, isEnd;
    protected double edgeValue;
    protected double accuracyPercent, usedTargetValue;

    protected boolean wasSmall = false;
    protected boolean wasBig = false;

    protected PFinding(
            boolean isSetControlNeedToRunPeriodic,
            double wantedAccuracyPercent, double timeoutForActionSeconds,
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

    protected void setIsInitTrue() {
        this.isInit = true;
        isEnd = false;
        isExe = false;
    }

    protected void setIsExecuteTrue() {
        this.isExe = true;
        isInit = false;
        isEnd = false;

    }

    protected void setIsEndTrue() {
        this.isEnd = true;
        isInit = false;
        isExe = false;
    }

    protected void replaceTargetValue() {
        usedTargetValue = (usedTargetValue == valuesToRunFor.getFirst()) ? valuesToRunFor.getSecond() : valuesToRunFor.getFirst();
    }

    protected double getAccuracyPercent(double error){
        return (100 - (100 / (accuracyRangeBestToWorst.getSecond() - accuracyRangeBestToWorst.getFirst() + 1)) * error);
    }

    @Override
    public void end(boolean interrupted) {
        stopAtEnd.run();
    }
}
