package frc.robot.constants;

import frc.utils.DriverStationUtils;
import frc.utils.allianceutils.AllianceTranslation2d;

public class FieldConstants {

    public static final double FIELD_LENGTH = 16.54175;

    public static final double FIELD_WIDTH = 8.0137;

    private static final AllianceTranslation2d SPEAKER = AllianceTranslation2d.fromBlueAllianceTranslation(0, 5);

    public static AllianceTranslation2d getAllianceSpeaker() {
        if (DriverStationUtils.isBlueAlliance()) {
            return AllianceTranslation2d.fromBlueAllianceTranslation(SPEAKER.getBlueAllianceTranslation2d());
        }
        return AllianceTranslation2d.fromBlueAllianceTranslation(SPEAKER.getMirroredAllianceTranslation2d());
    }

}
