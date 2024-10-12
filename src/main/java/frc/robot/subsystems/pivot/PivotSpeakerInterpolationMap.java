package frc.robot.subsystems.pivot;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;

public class PivotSpeakerInterpolationMap {

    public static InterpolatingTreeMap<Double, Double> DISTANCE_TO_ANGLE = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), Interpolator.forDouble()
    );

    static {
        DISTANCE_TO_ANGLE.put(0.9, Units.degreesToRadians(58));
        DISTANCE_TO_ANGLE.put(1.5, Units.degreesToRadians(48.5));
        DISTANCE_TO_ANGLE.put(2.0, Units.degreesToRadians(41));
        DISTANCE_TO_ANGLE.put(2.65, Units.degreesToRadians(38.2));
        DISTANCE_TO_ANGLE.put(2.7, Units.degreesToRadians(34.2));
        DISTANCE_TO_ANGLE.put(3.25, Units.degreesToRadians(33.75));
        DISTANCE_TO_ANGLE.put(3.47, Units.degreesToRadians(32.3));
        DISTANCE_TO_ANGLE.put(4.15, Units.degreesToRadians(28.9));
        DISTANCE_TO_ANGLE.put(4.5, Units.degreesToRadians(27.6));
        DISTANCE_TO_ANGLE.put(5.0, Units.degreesToRadians(26.2));
        DISTANCE_TO_ANGLE.put(5.5, Units.degreesToRadians(25));
    }

}
