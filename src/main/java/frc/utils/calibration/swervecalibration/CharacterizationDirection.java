package frc.utils.calibration.swervecalibration;

public enum CharacterizationDirection {

    CLOCKWISE(-1),
    COUNTER_CLOCKWISE(1);

    public final int directionSign;

    CharacterizationDirection(int directionSign) {
        this.directionSign = directionSign;
    }
}
