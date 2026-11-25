package frc.robot.orientation;

public enum OrientationSource {

    NONE(-1),
    IMU(0),
    SWERVE(1);

    private final int index;

    OrientationSource(int index){
        this.index = index;
    }

    public int getIndex() {
        return index;
    }

    public static OrientationSource getSourceByIndex(int index){
        return switch (index){
            case 0 -> IMU;
            case 1 -> SWERVE;
            default -> NONE;
        };
    }
}
