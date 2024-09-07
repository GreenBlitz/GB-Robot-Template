package frc.robot.poseestimator.limelight;

public enum LimelightArrayValue {
    X_AXIS,
    Y_AXIS,
    Z_AXIS,
    ROLL_ANGLE,
    PITCH_ANGLE,
    YAW_ANGLE,
    TOTAL_LATENCY;

    public int getValue() {
        return switch (this) {
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
