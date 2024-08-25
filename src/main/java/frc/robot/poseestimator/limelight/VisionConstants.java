package frc.robot.poseestimator.limelight;

import edu.wpi.first.math.geometry.Rotation2d;

public class VisionConstants {
    public static final String[] LIMELIGHT_NAMES = new String[]{
            "limelight-front",
            "limelight-back",
            "limelight-gb"
    };
    public static final Rotation2d ROTATION_TOLERANCE = Rotation2d.fromDegrees(40);

    public static final double POSITION_TOLERANCE = ;

    public static int LIMELIGHT_ENTRY_ARRAY_LENGTH = 7;

    public final static double VISION_TO_STANDARD_DEVIATION = 10;

    public final static double APRIL_TAG_HEIGHT_METERS = 1.2397;

    public final static double APRIL_TAG_HEIGHT_TOLERANCE_METERS = 0.07;

    public static enum LIMELIGHT_ARRAY_VALUES {
        X_AXIS,
        Y_AXIS,
        Z_AXIS,
        ROLL_ANGLE,
        PITCH_ANGLE,
        YAW_ANGLE,
        TOTAL_LATENCY;
    }

    public static int getValue(LIMELIGHT_ARRAY_VALUES value) {
        int arrayValue = -1;
        switch (value) {
            case X_AXIS -> {
                arrayValue = 0;
            }
            case Y_AXIS -> {
                arrayValue = 1;
            }
            case Z_AXIS -> {
                arrayValue = 2;
            }
            case ROLL_ANGLE -> {
                arrayValue = 3;
            }
            case YAW_ANGLE -> {
                arrayValue = 4;
            }
            case PITCH_ANGLE -> {
                arrayValue = 5;
            }
            case TOTAL_LATENCY -> {
                arrayValue = 6;
            }
        }
        return arrayValue;
    }
}
