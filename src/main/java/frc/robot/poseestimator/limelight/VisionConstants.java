package frc.robot.poseestimator.limelight;

import edu.wpi.first.math.geometry.Rotation2d;

public class VisionConstants {

    public static final String[] LIMELIGHT_NAMES = new String[]{
            "limelight-front",
            "limelight-back",
            "limelight-gb"
    };

    public static final String LIMELIGHT_LOGPATH_PREFIX = "Limelight ";

    public static final String MULTI_LIMELIGHT_LOGPATH = "LimelightsHardware/";

    public static final String LIMELIGHTS_FILTERED_LOGPATH = "LimelightsFiltered/";

    public static final String ESTIMATION_LOGPATH_PREFIX = "estimation ";

    //! shall be calibrated
    public static final Rotation2d ROTATION_TOLERANCE = Rotation2d.fromDegrees(20);

    //! shall be calibrated
    public static final double POSITION_TOLERANCE = 0.2;

    public static int LIMELIGHT_ENTRY_ARRAY_LENGTH = 7;

    public final static double VISION_TO_STANDARD_DEVIATION = 10;

    public final static double APRIL_TAG_HEIGHT_METERS = 1.2397;

    public final static double APRIL_TAG_HEIGHT_TOLERANCE_METERS = 0.07;


    public enum LIMELIGHT_ARRAY_VALUE {
        X_AXIS,
        Y_AXIS,
        Z_AXIS,
        ROLL_ANGLE,
        PITCH_ANGLE,
        YAW_ANGLE,
        TOTAL_LATENCY;
    }

    public static int getValue(LIMELIGHT_ARRAY_VALUE value) {
        return switch (value) {
            case X_AXIS -> 0;
            case Y_AXIS -> 1;
            case Z_AXIS -> 2;
            case ROLL_ANGLE -> 3;
            case YAW_ANGLE -> 4;
            case PITCH_ANGLE -> 5;
            case TOTAL_LATENCY -> 6;
        };
    }


}
