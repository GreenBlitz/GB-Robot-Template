package frc.utils.calibration.autocalibration.kpfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;
import frc.utils.commands.GBCommand;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;

public abstract class PFinding extends GBCommand {

    protected final boolean isSetControlNeedToRunPeriodic;
    protected final double tolerance, timeoutForActionSeconds;
    protected final Pair<Double, Double> valuesToRunFor;
    protected final DoubleSupplier currentValueSupplier, currentKpValueSupplier;
    protected final Consumer<Double> setControl, setKp;
    protected final Predicate<Double> isAtPose;
    protected final Runnable stopAtEnd;

    protected final Timer TIMER;
    protected boolean isInit, isExe, isEnd;
    protected double usedTargetValue;
    protected double error;

    protected PFinding(
            boolean isSetControlNeedToRunPeriodic,
            double tolerance, double timeoutForActionSeconds,
            Pair<Double, Double> valuesToRunFor,
            DoubleSupplier currentValueSupplier, DoubleSupplier currentKpValueSupplier,
            Consumer<Double> setControl, Consumer<Double> setKp,
            Predicate<Double> isAtPose,
            Runnable stopAtEnd
    ) {
        this.TIMER = new Timer();

        this.isSetControlNeedToRunPeriodic = isSetControlNeedToRunPeriodic;

        this.tolerance = tolerance;
        this.timeoutForActionSeconds = timeoutForActionSeconds;

        this.valuesToRunFor = valuesToRunFor;
        this.usedTargetValue = valuesToRunFor.getSecond();

        this.currentValueSupplier = currentValueSupplier;
        this.currentKpValueSupplier = currentKpValueSupplier;

        this.setControl = setControl;
        this.setKp = setKp;
        this.isAtPose = isAtPose;
        this.stopAtEnd = stopAtEnd;
    }

    @Override
    public void initialize() {
        setIsInitTrue();
    }

    @Override
    public void end(boolean interrupted) {
        stopAtEnd.run();
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

    protected boolean hasOscillated(double currentPosition) {
        return !((valuesToRunFor.getFirst() >= currentPosition && currentPosition >= valuesToRunFor.getSecond())
                || (valuesToRunFor.getFirst() <= currentPosition && currentPosition <= valuesToRunFor.getSecond()));
    }

    protected void initFunction(){
        TIMER.restart();
        replaceTargetValue();
        error = Math.abs(currentValueSupplier.getAsDouble() - usedTargetValue);
        setControl.accept(usedTargetValue);
        setIsExecuteTrue();
    }

    protected void setControlPeriodic(){
        if (isSetControlNeedToRunPeriodic) {
            setControl.accept(usedTargetValue);
        }
    }

    protected boolean isNeedToBeEnd(double currentPosition){
        return isAtPose.test(usedTargetValue) || TIMER.hasElapsed(timeoutForActionSeconds) || hasOscillated(currentPosition);
    }
}
