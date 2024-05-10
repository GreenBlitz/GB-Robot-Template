package frc.utils.allianceutils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.MathConstants;

import static frc.utils.DriverStationUtils.isBlueAlliance;

public class AllianceUtils {

    /**
     * Converts a pose to the pose relative to the current driver station alliance's right corner.
     *
     * @param pose the current blue alliance pose
     * @return the converted pose
     */
    protected static Pose2d toAlliancePose(Pose2d pose) {
        return isBlueAlliance() ? pose : switchAlliance(pose);
    }

    protected static Translation2d toAllianceTranslation(Translation2d translation2d) {
        return isBlueAlliance() ? translation2d : switchAlliance(translation2d);
    }

    protected static Rotation2d toAllianceAngle(Rotation2d angle) {
        return isBlueAlliance() ? angle : switchAlliance(angle);
    }

    private static Pose2d switchAlliance(Pose2d pose) {
        return new Pose2d(
                switchAlliance(pose.getTranslation()),
                switchAlliance(pose.getRotation())
        );
    }

    private static Translation2d switchAlliance(Translation2d translation2d) {
        return new Translation2d(
                FieldConstants.FIELD_LENGTH - translation2d.getX(),
                FieldConstants.FIELD_WIDTH - translation2d.getY()
        );
    }

    private static Rotation2d switchAlliance(Rotation2d rotation2d) {
        return rotation2d.minus(MathConstants.HALF_CIRCLE);
    }

    /**
     * Mirrors a pose across the center of the field if the current alliance is red.
     *
     * @param pose the pose to mirror if the current alliance is red
     * @return the pose
     */
    protected static Pose2d toMirroredAlliancePose(Pose2d pose) {
        return isBlueAlliance() ? pose : mirror(pose);
    }

    protected static Translation2d toMirroredAllianceTranslation(Translation2d translation2d) {
        return isBlueAlliance() ? translation2d : mirror(translation2d);
    }

    protected static Rotation2d toMirroredAllianceAngle(Rotation2d angle) {
        return isBlueAlliance() ? angle : mirror(angle);
    }

    private static Pose2d mirror(Pose2d pose) {
        return new Pose2d(
                mirror(pose.getTranslation()),
                mirror(pose.getRotation())
        );
    }

    private static Translation2d mirror(Translation2d translation2d) {
        return new Translation2d(
                FieldConstants.FIELD_LENGTH - translation2d.getX(),
                translation2d.getY()
        );
    }

    private static Rotation2d mirror(Rotation2d rotation2d) {
        return MathConstants.HALF_CIRCLE.minus(rotation2d);
    }

}
