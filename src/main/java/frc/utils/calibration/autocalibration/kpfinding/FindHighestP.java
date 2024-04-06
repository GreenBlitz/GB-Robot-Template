package frc.utils.calibration.autocalibration.kpfinding;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Predicate;

public class FindHighestP extends SequentialCommandGroup {

    /**
     * This function is finding for you what is the best (biggest) kP value to use on your system.
     * @author Yoav Herman
     * <p>
     * IMPORTANT:
     * @apiNote You can choose what units to use or which control mode you calibrate. BUT Make sure you use the SAME UNITS EVERYWHERE!!!
     * @apiNote This function must get RELATIVE values and NOT ABSOLUTE
     *
     * @param isSetControlNeedToRunPeriodic -> is the setControl func run on the motor or on the rio
     * @param wantedAccuracyPercent -> wanted level of accuracy
     * @param timeoutForActionSeconds -> max time for action
     * @param errorToKpValueFactor -> how many kP to add or minus for one error
     * @param multiPFactor -> by how much increase kP until there is Oscillate
     * @param valuesToRunFor -> two points og the system to jump between
     * @param accuracyRangeBestToWorst -> TODO
     * @param currentValueSupplier -> supplier to the current position of the system
     * @param currentKpValueSupplier -> supplier to current kP value
     * @param setControl -> function to control the system
     * @param setKp -> function to set new kP
     * @param isAtPose -> function to check is system at position
     * @param stopAtEnd -> function to do in and (probably stand in place)
     */

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
