package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.DriverStationUtils;
import frc.utils.allianceutils.AllianceRotation2d;
import frc.utils.allianceutils.AllianceTranslation2d;

public class FieldConstants {

    public static final double FIELD_LENGTH = 16.54175;

    public static final double FIELD_WIDTH = 8.0137;

    private static final AllianceTranslation2d SPEAKER = AllianceTranslation2d.fromBlueAllianceTranslation(
            0.23,
            FIELD_WIDTH - 2.55
    );

    public static AllianceTranslation2d getAllianceSpeaker() {
        if (DriverStationUtils.isBlueAlliance()) {
            return AllianceTranslation2d.fromBlueAllianceTranslation(SPEAKER.getBlueAllianceTranslation2d());
        }
        return AllianceTranslation2d.fromBlueAllianceTranslation(SPEAKER.getMirroredAllianceTranslation2d());
    }

    public static final AllianceRotation2d ANGLE_TO_AMP = AllianceRotation2d.fromBlueAllianceRotation(Rotation2d.fromDegrees(90));

}
