package frc.utils.calibration.autocalibration.kpfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;

public class FindHighestP extends SequentialCommandGroup {

    public FindHighestP(
            boolean isSetControlNeedToRunPeriodic,
            double wantedAccuracyPercent, double timeoutForActionSeconds, double errorToKpValueFactor, double multiPFactor,
            Pair<Double, Double> valuesToRunFor, Pair<Double, Double> accuracyRangeBestToWorst,
            DoubleSupplier currentValueSupplier, DoubleSupplier currentKpValueSupplier,
            Consumer<Double> setControl, Consumer<Double> setKp,
            Predicate<Double> isAtPose,
            Runnable stopAtEnd
    ) {
        super(
                new FindP(
                        isSetControlNeedToRunPeriodic,
                        wantedAccuracyPercent, timeoutForActionSeconds, errorToKpValueFactor,
                        valuesToRunFor, accuracyRangeBestToWorst,
                        currentValueSupplier, currentKpValueSupplier,
                        setControl, setKp,
                        isAtPose,
                        stopAtEnd
                ),
                new FindMaxPBeforeOscillate(
                        isSetControlNeedToRunPeriodic,
                        wantedAccuracyPercent, timeoutForActionSeconds, multiPFactor,
                        valuesToRunFor, accuracyRangeBestToWorst,
                        currentValueSupplier, currentKpValueSupplier,
                        setControl, setKp,
                        isAtPose,
                        stopAtEnd
                )
        );
    }

}
