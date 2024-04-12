package frc.utils.allianceutils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AlliancePose2d {

    private final Pose2d blueAlliancePose;
    private final Pose2d alliancePose;
    private final Pose2d mirroredAlliancePose;

    private AlliancePose2d(Pose2d blueAlliancePose, Pose2d alliancePose, Pose2d mirroredAlliancePose) {
        this.blueAlliancePose = blueAlliancePose;
        this.alliancePose = alliancePose;
        this.mirroredAlliancePose = mirroredAlliancePose;
    }

    public static AlliancePose2d fromBlueAlliancePose(Translation2d translation, Rotation2d rotation) {
        return fromBlueAlliancePose(new Pose2d(translation, rotation));
    }

    public static AlliancePose2d fromBlueAlliancePose(Pose2d blueAlliancePose) {
        return new AlliancePose2d(
                blueAlliancePose,
                AllianceUtils.toAlliancePose(blueAlliancePose),
                AllianceUtils.toMirroredAlliancePose(blueAlliancePose)
        );
    }

    public static AlliancePose2d fromBlueAlliancePose(double x, double y, Rotation2d rotation) {
        return fromBlueAlliancePose(new Pose2d(x, y, rotation));
    }

    public static AlliancePose2d fromAlliancePose(Translation2d translation, Rotation2d rotation) {
        return fromAlliancePose(new Pose2d(translation, rotation));
    }

    public static AlliancePose2d fromAlliancePose(Pose2d alliancePose) {
        final Pose2d blueAlliancePose = AllianceUtils.toAlliancePose(alliancePose);
        return new AlliancePose2d(blueAlliancePose, alliancePose, AllianceUtils.toMirroredAlliancePose(blueAlliancePose));
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
