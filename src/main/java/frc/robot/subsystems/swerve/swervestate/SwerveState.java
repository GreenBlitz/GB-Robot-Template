package frc.robot.subsystems.swerve.swervestate;

public class SwerveState {

    public static final DriveMode DEFAULT_DRIVE_MODE = DriveMode.FIELD_RELATIVE;
    public static final DriveSpeed DEFAULT_DRIVE_SPEED = DriveSpeed.NORMAL;
    public static final LoopMode DEFAULT_LOOP_MODE = LoopMode.OPEN;
    public static final RotateAxis DEFAULT_ROTATE_AXIS = RotateAxis.MIDDLE_OF_ROBOT;


    private DriveMode driveMode;
    private DriveSpeed driveSpeed;
    private LoopMode loopMode;
    private RotateAxis rotateAxis;

    public SwerveState() {
        this(DEFAULT_DRIVE_MODE, DEFAULT_DRIVE_SPEED, DEFAULT_LOOP_MODE, DEFAULT_ROTATE_AXIS);
    }

    public SwerveState(DriveMode driveMode, DriveSpeed driveSpeed, LoopMode loopMode, RotateAxis rotateAxis) {
        this.driveMode = driveMode;
        this.driveSpeed = driveSpeed;
        this.loopMode = loopMode;
        this.rotateAxis = rotateAxis;
    }

    public void updateState(SwerveState newState) {
        driveSpeed = newState.driveSpeed;
        loopMode = newState.loopMode;
        rotateAxis = newState.rotateAxis;
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    public DriveSpeed getDriveSpeed() {
        return driveSpeed;
    }

    public LoopMode getLoopMode() {
        return loopMode;
    }

    public RotateAxis getRotateAxis() {
        return rotateAxis;
    }

}
