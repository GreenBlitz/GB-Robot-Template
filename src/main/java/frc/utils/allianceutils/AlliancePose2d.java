package frc.utils.allianceutils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AlliancePose2d {

    private final AllianceTranslation2d allianceTranslation2d;

    private final AllianceRotation2d allianceRotation2d;

    private AlliancePose2d(AllianceTranslation2d allianceTranslation2d, AllianceRotation2d allianceRotation2d) {
        this.allianceTranslation2d = allianceTranslation2d;
        this.allianceRotation2d = allianceRotation2d;
    }

    public static AlliancePose2d fromBlueAlliancePose(Translation2d translation, Rotation2d rotation) {
        return new AlliancePose2d(
                AllianceTranslation2d.fromBlueAllianceTranslation(translation),
                AllianceRotation2d.fromBlueAllianceRotation(rotation)
        );
    }

    public static AlliancePose2d fromBlueAlliancePose(Pose2d blueAlliancePose) {
        return fromBlueAlliancePose(
                blueAlliancePose.getTranslation(),
                blueAlliancePose.getRotation()
        );
    }

    public static AlliancePose2d fromBlueAlliancePose(double x, double y, Rotation2d rotation) {
        return fromBlueAlliancePose(new Pose2d(x, y, rotation));
    }

    public static AlliancePose2d fromAlliancePose(Translation2d translation, Rotation2d rotation) {
        return new AlliancePose2d(
                AllianceTranslation2d.fromAllianceTranslation(translation),
                AllianceRotation2d.fromAllianceRotation(rotation)
        );
    }

    public static AlliancePose2d fromAlliancePose(Pose2d alliancePose) {
        return fromAlliancePose(alliancePose.getTranslation(), alliancePose.getRotation());
    }

    public static AlliancePose2d fromAlliancePose(double x, double y, Rotation2d rotation) {
        return fromAlliancePose(new Pose2d(x, y, rotation));
    }

    public Pose2d getBlueAlliancePose() {
        return new Pose2d(allianceTranslation2d.getBlueAllianceTranslation2d(), allianceRotation2d.getBlueAllianceAngle());
    }

    public Pose2d getAlliancePose() {
        return new Pose2d(allianceTranslation2d.getAllianceTranslation2d(), allianceRotation2d.getAllianceAngle());
    }

    public Pose2d getMirroredAlliancePose() {
        return new Pose2d(
                allianceTranslation2d.getMirroredAllianceTranslation2d(),
                allianceRotation2d.getMirroredAllianceAngle()
        );
    }

    public double getDistance(AlliancePose2d alliancePose2d) {
        return allianceTranslation2d.getDistance(alliancePose2d.allianceTranslation2d);
    }

}
