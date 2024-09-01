package frc.utils;

import frc.robot.constants.FieldConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class AllianceUtilities {

    private static DriverStation.Alliance ALLIANCE = DriverStation.Alliance.Blue;

    private static double BLUE_ALLIANCE_CHECK_TIMESTAMP = -1;

    /**
     * @return whether the robot is on the blue alliance
     */
    public static boolean isBlueAlliance() {
        final double timestamp = Timer.getFPGATimestamp();
        if (timestamp - BLUE_ALLIANCE_CHECK_TIMESTAMP > FieldConstants.MINIMUM_TIMESTAMP_ALLIANCE_CHECK) {
            ALLIANCE = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
            BLUE_ALLIANCE_CHECK_TIMESTAMP = timestamp;
        }
        return ALLIANCE == DriverStation.Alliance.Blue;
    }

    /**
     * Converts a pose to the pose relative to the current driver station alliance's right corner.
     *
     * @param pose the current blue alliance pose
     * @return the converted pose
     */
    public static Pose2d toAlliancePose(Pose2d pose) {
        return isBlueAlliance() ? pose : switchAlliance(pose);
    }

    /**
     * Mirrors a pose across the center of the field if the current alliance is red.
     *
     * @param pose the pose to mirror if the current alliance is red
     * @return the pose
     */
    public static Pose2d toMirroredAlliancePose(Pose2d pose) {
        return isBlueAlliance() ? pose : mirror(pose);
    }

    /**
     * Mirrors a rotation across the center of the field if the current alliance is red.
     *
     * @param rotation the rotation to mirror if the current alliance is red
     * @return the rotation
     */
    public static Rotation2d toMirroredAllianceRotation(Rotation2d rotation) {
        return isBlueAlliance() ? rotation : FieldConstants.FULL_CYCLE.minus(rotation);
    }

    private static Pose2d mirror(Pose2d pose) {
        return new Pose2d(
                FieldConstants.LENGTH_METERS - pose.getX(),
                pose.getY(),
                new Rotation2d(Math.PI).minus(pose.getRotation())
        );
    }

    private static Pose2d switchAlliance(Pose2d pose) {
        return new Pose2d(
                FieldConstants.LENGTH_METERS - pose.getX(),
                FieldConstants.WIDTH_METERS - pose.getY(),
                pose.getRotation().minus(FieldConstants.HALF_CYCLE)
        );
    }

    public static class AlliancePose2d {

        private final Pose2d blueAlliancePose;
        private final Pose2d alliancePose;
        private final Pose2d mirroredAlliancePose;

        private AlliancePose2d(Pose2d blueAlliancePose, Pose2d alliancePose, Pose2d mirroredAlliancePose) {
            this.blueAlliancePose = blueAlliancePose;
            this.alliancePose = alliancePose;
            this.mirroredAlliancePose = mirroredAlliancePose;
        }

        public static AlliancePose2d fromBlueAlliancePose(Pose2d blueAlliancePose) {
            return new AlliancePose2d(blueAlliancePose, AllianceUtilities.toAlliancePose(blueAlliancePose), AllianceUtilities.toMirroredAlliancePose(blueAlliancePose));
        }

        public static AlliancePose2d fromBlueAlliancePose(Translation2d translation, Rotation2d rotation) {
            return fromBlueAlliancePose(new Pose2d(translation, rotation));
        }

        public static AlliancePose2d fromBlueAlliancePose(double x, double y, Rotation2d rotation) {
            return fromBlueAlliancePose(new Pose2d(x, y, rotation));
        }

        public static AlliancePose2d fromAlliancePose(Pose2d alliancePose) {
            final Pose2d blueAlliancePose = AllianceUtilities.toAlliancePose(alliancePose);
            return new AlliancePose2d(blueAlliancePose, alliancePose, AllianceUtilities.toMirroredAlliancePose(blueAlliancePose));
        }

        public static AlliancePose2d fromAlliancePose(Translation2d translation, Rotation2d rotation) {
            return fromAlliancePose(new Pose2d(translation, rotation));
        }

        public static AlliancePose2d fromAlliancePose(double x, double y, Rotation2d rotation) {
            return fromAlliancePose(new Pose2d(x, y, rotation));
        }

        public Pose2d toBlueAlliancePose() {
            return blueAlliancePose;
        }

        public Pose2d toAlliancePose() {
            return alliancePose;
        }

        public Pose2d toMirroredAlliancePose() {
            return mirroredAlliancePose;
        }


    }


}
