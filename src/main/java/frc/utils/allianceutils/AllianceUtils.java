package frc.utils.allianceutils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.FieldConstants;

import static frc.utils.DriverStationUtils.isBlueAlliance;

public class AllianceUtils {

    /**
     * Converts a pose to the pose relative to the current driver station alliance's right corner.
     *
     * @param pose the current blue alliance pose
     * @return the converted pose
     */
    public static Pose2d toAlliancePose(Pose2d pose) {
        if (isBlueAlliance())
            return pose;
        return switchAlliance(pose);
    }

    /**
     * Mirrors a pose across the center of the field if the current alliance is red.
     *
     * @param pose the pose to mirror if the current alliance is red
     * @return the pose
     */
    public static Pose2d toMirroredAlliancePose(Pose2d pose) {
        if (isBlueAlliance())
            return pose;
        return mirror(pose);
    }

    /**
     * Mirrors a rotation across the center of the field if the current alliance is red.
     *
     * @param rotation the rotation to mirror if the current alliance is red
     * @return the rotation
     */
    public static Rotation2d toMirroredAllianceRotation(Rotation2d rotation) {
        if (isBlueAlliance())
            return rotation;
        return new Rotation2d(Math.PI).minus(rotation);
    }

    private static Pose2d mirror(Pose2d pose) {
        return new Pose2d(
                FieldConstants.FIELD_LENGTH - pose.getX(),
                pose.getY(),
                new Rotation2d(Math.PI).minus(pose.getRotation())
        );
    }

    private static Pose2d switchAlliance(Pose2d pose) {
        return new Pose2d(
                FieldConstants.FIELD_LENGTH - pose.getX(),
                FieldConstants.FIELD_WIDTH - pose.getY(),
                pose.getRotation().minus(Rotation2d.fromRotations(0.5))
        );
    }
}
