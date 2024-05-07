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

    public SwerveState(DriveMode driveMode) {
        this(driveMode, DEFAULT_DRIVE_SPEED, DEFAULT_LOOP_MODE, DEFAULT_ROTATE_AXIS);
    }

    public SwerveState(DriveSpeed driveSpeed) {
        this(DEFAULT_DRIVE_MODE, driveSpeed, DEFAULT_LOOP_MODE, DEFAULT_ROTATE_AXIS);
    }

    public SwerveState(LoopMode loopMode) {
        this(DEFAULT_DRIVE_MODE, DEFAULT_DRIVE_SPEED, loopMode, DEFAULT_ROTATE_AXIS);
    }

    public SwerveState(RotateAxis rotateAxis) {
        this(DEFAULT_DRIVE_MODE, DEFAULT_DRIVE_SPEED, DEFAULT_LOOP_MODE, rotateAxis);
    }

    public SwerveState(DriveMode driveMode, DriveSpeed driveSpeed) {
        this(driveMode, driveSpeed, DEFAULT_LOOP_MODE, DEFAULT_ROTATE_AXIS);
    }

    public SwerveState(DriveMode driveMode, LoopMode loopMode) {
        this(driveMode, DEFAULT_DRIVE_SPEED, loopMode, DEFAULT_ROTATE_AXIS);
    }

    public SwerveState(DriveMode driveMode, RotateAxis rotateAxis) {
        this(driveMode, DEFAULT_DRIVE_SPEED, DEFAULT_LOOP_MODE, rotateAxis);
    }

    public SwerveState(DriveSpeed driveSpeed, LoopMode loopMode) {
        this(DEFAULT_DRIVE_MODE, driveSpeed, loopMode, DEFAULT_ROTATE_AXIS);
    }

    public SwerveState(DriveSpeed driveSpeed, RotateAxis rotateAxis) {
        this(DEFAULT_DRIVE_MODE, driveSpeed, DEFAULT_LOOP_MODE, rotateAxis);
    }

    public SwerveState(LoopMode loopMode, RotateAxis rotateAxis) {
        this(DEFAULT_DRIVE_MODE, DEFAULT_DRIVE_SPEED, loopMode, rotateAxis);
    }

    public SwerveState(DriveMode driveMode, DriveSpeed driveSpeed, LoopMode loopMode) {
        this(driveMode, driveSpeed, loopMode, DEFAULT_ROTATE_AXIS);
    }

    public SwerveState(DriveMode driveMode, DriveSpeed driveSpeed, RotateAxis rotateAxis) {
        this(driveMode, driveSpeed, DEFAULT_LOOP_MODE, rotateAxis);
    }

    public SwerveState(DriveMode driveMode, LoopMode loopMode, RotateAxis rotateAxis) {
        this(driveMode, DEFAULT_DRIVE_SPEED, loopMode, rotateAxis);
    }

    public SwerveState(DriveSpeed driveSpeed, LoopMode loopMode, RotateAxis rotateAxis) {
        this(DEFAULT_DRIVE_MODE, driveSpeed, loopMode, rotateAxis);
    }

    public SwerveState(DriveMode driveMode, DriveSpeed driveSpeed, LoopMode loopMode, RotateAxis rotateAxis) {
        this.driveMode = driveMode;
        this.driveSpeed = driveSpeed;
        this.loopMode = loopMode;
        this.rotateAxis = rotateAxis;
    }

    public void updateState(SwerveState newState) {
        driveMode = newState.driveMode;
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
