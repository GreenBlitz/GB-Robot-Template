package frc.utils.allianceutils;

import edu.wpi.first.math.geometry.Translation2d;

public class AllianceTranslation2d {

    private final Translation2d blueAllianceTranslation2d;

    private final Translation2d allianceTranslation2d;

    private final Translation2d mirroredAllianceTranslation2d;

    private AllianceTranslation2d(Translation2d blueAllianceTranslation2d, Translation2d allianceTranslation2d,
            Translation2d mirroredAllianceTranslation2d) {
        this.blueAllianceTranslation2d = blueAllianceTranslation2d;
        this.allianceTranslation2d = allianceTranslation2d;
        this.mirroredAllianceTranslation2d = mirroredAllianceTranslation2d;
    }

    public static AllianceTranslation2d fromBlueAllianceTranslation(Translation2d blueAllianceTranslation) {
        return new AllianceTranslation2d(
                blueAllianceTranslation,
                AllianceUtils.toAllianceTranslation(blueAllianceTranslation),
                AllianceUtils.toMirroredAllianceTranslation(blueAllianceTranslation)
        );
    }

    public static AllianceTranslation2d fromBlueAllianceTranslation(double x, double y) {
        return fromBlueAllianceTranslation(new Translation2d(x, y));
    }

    public static AllianceTranslation2d fromAlliancePose(Translation2d allianceTranslation) {
        Translation2d blueAlliancePose = AllianceUtils.toAllianceTranslation(allianceTranslation);
        return new AllianceTranslation2d(
                blueAlliancePose,
                allianceTranslation,
                AllianceUtils.toMirroredAllianceTranslation(allianceTranslation)
        );
    }

    public static AllianceTranslation2d fromAlliancePose(double x, double y) {
        return fromAlliancePose(new Translation2d(x, y));
    }

    public Translation2d getBlueAllianceTranslation2d() {
        return blueAllianceTranslation2d;
    }

    public Translation2d getAllianceTranslation2d() {
        return allianceTranslation2d;
    }

    public Translation2d getMirroredAllianceTranslation2d() {
        return mirroredAllianceTranslation2d;
    }

}
