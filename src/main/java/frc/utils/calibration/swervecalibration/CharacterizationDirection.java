package frc.utils.calibration.swervecalibration;

public enum CharacterizationDirection {

    CLOCKWISE(-1),
    COUNTER_CLOCKWISE(1);

    private final int directionSign;

    CharacterizationDirection(int directionSign) {
        this.directionSign = directionSign;
    }

    public int getDirectionSign() {
        return directionSign;
    }
}
