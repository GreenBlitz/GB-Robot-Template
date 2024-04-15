package frc.utils.calibration.autocalibration.findkp;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;
import frc.utils.GBSubsystem;
import frc.utils.commands.GBCommand;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;

public abstract class KpFindingUtil extends GBCommand {

    protected final Pair<Double, Double> valuesToRunFor;

    protected final DoubleSupplier currentValueSupplier, currentKpValueSupplier;

    protected final Consumer<Double> setControl, setKp;

    protected final Predicate<Double> isAtPose;

    protected final Runnable stopAtEnd;

    protected final boolean isSetControlNeedToRunPeriodic;

    protected final double tolerance, timeoutForActionSeconds;

    protected final Timer TIMER;

    protected boolean isInitialize, isExecute, isEnd;

    protected double usedTargetValue;

    protected double error;

    protected KpFindingUtil(GBSubsystem subsystem, boolean isSetControlNeedToRunPeriodic, double tolerance,
            double timeoutForActionSeconds, Pair<Double, Double> valuesToRunFor, DoubleSupplier currentValueSupplier,
            DoubleSupplier currentKpValueSupplier, Consumer<Double> setControl, Consumer<Double> setKp,
            Predicate<Double> isAtPose, Runnable doOnEnd) {
        this.TIMER = new Timer();

        this.isSetControlNeedToRunPeriodic = isSetControlNeedToRunPeriodic;

        this.tolerance = tolerance;
        this.timeoutForActionSeconds = timeoutForActionSeconds;

        this.valuesToRunFor = valuesToRunFor;

        this.currentValueSupplier = currentValueSupplier;
        this.currentKpValueSupplier = currentKpValueSupplier;

        this.setControl = setControl;
        this.setKp = setKp;
        this.isAtPose = isAtPose;
        this.stopAtEnd = doOnEnd;

        require(subsystem);
    }

    @Override
    public void initialize() {
        setIsInitTrue();
        setUsedTargetValueToFurtherMostValue();
    }

    @Override
    public void end(boolean interrupted) {
        stopAtEnd.run();
    }

    public void setUsedTargetValueToFurtherMostValue() {
        final double currentValue = currentValueSupplier.getAsDouble();
        if (Math.abs(currentValue - valuesToRunFor.getFirst()) > Math.abs(currentValue - valuesToRunFor.getSecond())) {
            usedTargetValue = valuesToRunFor.getFirst();
        }
        else {
            usedTargetValue = valuesToRunFor.getSecond();
        }
    }

    protected void setIsInitTrue() {
        this.isInitialize = true;
        isEnd = false;
        isExecute = false;
    }

    protected void setIsExecuteTrue() {
        this.isExecute = true;
        isInitialize = false;
        isEnd = false;
    }

    protected void setIsEndTrue() {
        this.isEnd = true;
        isInitialize = false;
        isExecute = false;
    }

    protected void replaceTargetValue() {
        usedTargetValue = (usedTargetValue == valuesToRunFor.getFirst()) ? valuesToRunFor.getSecond() : valuesToRunFor.getFirst();
    }

    protected boolean hasOscillated(double currentPosition) {
        return !((valuesToRunFor.getFirst() + tolerance >= currentPosition && currentPosition >= valuesToRunFor.getSecond() - tolerance) || (valuesToRunFor.getFirst() - tolerance <= currentPosition && currentPosition <= valuesToRunFor.getSecond() + tolerance));
    }

    protected void initFunction() {
        TIMER.restart();
        replaceTargetValue();
        error = Math.abs(currentValueSupplier.getAsDouble() - usedTargetValue);
        setControl.accept(usedTargetValue);
        setIsExecuteTrue();
    }

    protected void setControlPeriodic() {
        if (isSetControlNeedToRunPeriodic) {
            setControl.accept(usedTargetValue);
        }
    }

    protected boolean isNeedToBeEnd(double currentPosition) {
        return isAtPose.test(usedTargetValue) || TIMER.hasElapsed(timeoutForActionSeconds) || hasOscillated(currentPosition);
    }

}