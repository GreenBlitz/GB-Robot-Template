package frc.robot.subsystems.swerve;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DiscritizationHelper {

    private final InterpolatingDoubleTreeMap linearInterpolationMap;
    private final InterpolatingDoubleTreeMap angularInterpolationMap;

    public DiscritizationHelper(InterpolatingDoubleTreeMap linearInterpolationMap, InterpolatingDoubleTreeMap angularInterpolationMap) {
        this.linearInterpolationMap = linearInterpolationMap;
        this.angularInterpolationMap = angularInterpolationMap;
    }

    public double getDiscritizationFactor (ChassisSpeeds speeds){
        return  linearInterpolationMap.get(SwerveMath.getDriveMagnitude(speeds)) + angularInterpolationMap.get(speeds.omegaRadiansPerSecond);
    }
}
