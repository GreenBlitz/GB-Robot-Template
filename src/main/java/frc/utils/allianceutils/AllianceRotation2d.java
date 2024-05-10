package frc.utils.allianceutils;

import edu.wpi.first.math.geometry.Rotation2d;

public class AllianceRotation2d {

    private final Rotation2d blueAllianceAngle;

    private final Rotation2d allianceAngle;

    private final Rotation2d mirroredAllianceAngle;

    private AllianceRotation2d(Rotation2d blueAllianceAngle, Rotation2d allianceAngle, Rotation2d mirroredAllianceAngle) {
        this.blueAllianceAngle = blueAllianceAngle;
        this.allianceAngle = allianceAngle;
        this.mirroredAllianceAngle = mirroredAllianceAngle;
    }

    public static AllianceRotation2d fromBlueAllianceRotation(Rotation2d blueAllianceAngle) {
        return new AllianceRotation2d(
                blueAllianceAngle,
                AllianceUtils.toAllianceAngle(blueAllianceAngle),
                AllianceUtils.toMirroredAllianceAngle(blueAllianceAngle)
        );
    }

    public static AllianceRotation2d fromAllianceRotation(Rotation2d allianceAngle) {
        Rotation2d blueAllianceAngle = AllianceUtils.toAllianceAngle(allianceAngle);
        return new AllianceRotation2d(
                blueAllianceAngle,
                allianceAngle,
                AllianceUtils.toMirroredAllianceAngle(blueAllianceAngle)
        );
    }

    public Rotation2d getBlueAllianceAngle() {
        return blueAllianceAngle;
    }

    public Rotation2d getAllianceAngle() {
        return allianceAngle;
    }

    public Rotation2d getMirroredAllianceAngle() {
        return mirroredAllianceAngle;
    }

}
