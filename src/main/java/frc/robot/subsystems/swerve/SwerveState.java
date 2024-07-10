package frc.robot.subsystems.swerve;

import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import frc.robot.subsystems.swerve.swervestatehelpers.DriveRelative;
import frc.robot.subsystems.swerve.swervestatehelpers.DriveSpeed;
import frc.robot.subsystems.swerve.swervestatehelpers.LoopMode;
import frc.robot.subsystems.swerve.swervestatehelpers.RotateAxis;

public class SwerveState {

    private static final DriveRelative DEFAULT_DRIVE_RELATIVE = DriveRelative.FIELD_RELATIVE;
    private static final DriveSpeed DEFAULT_DRIVE_SPEED = DriveSpeed.NORMAL;
    private static final LoopMode DEFAULT_LOOP_MODE = LoopMode.OPEN; // todo - move to close probably (ks for couple handel)
    private static final RotateAxis DEFAULT_ROTATE_AXIS = RotateAxis.MIDDLE_OF_ROBOT;
    private static final AimAssist DEFAULT_AIM_ASSIST = AimAssist.NONE;

    public static final SwerveState DEFAULT_PATH_PLANNER = new SwerveState().withDriveMode(DriveRelative.ROBOT_RELATIVE);
    public static final SwerveState DEFAULT_DRIVE = new SwerveState();


    private DriveRelative driveRelative;
    private DriveSpeed driveSpeed;
    private LoopMode loopMode;
    private RotateAxis rotateAxis;
    private AimAssist aimAssist;

    public SwerveState(SwerveState swerveState) {
        this(swerveState.driveRelative, swerveState.driveSpeed, swerveState.loopMode, swerveState.rotateAxis, swerveState.aimAssist);
    }

    private SwerveState() {
        this(DEFAULT_DRIVE_RELATIVE, DEFAULT_DRIVE_SPEED, DEFAULT_LOOP_MODE, DEFAULT_ROTATE_AXIS, DEFAULT_AIM_ASSIST);
    }

    private SwerveState(DriveRelative driveRelative, DriveSpeed driveSpeed, LoopMode loopMode, RotateAxis rotateAxis, AimAssist aimAssist) {
        this.driveRelative = driveRelative;
        this.driveSpeed = driveSpeed;
        this.loopMode = loopMode;
        this.rotateAxis = rotateAxis;
        this.aimAssist = aimAssist;
    }


    public SwerveState withDriveMode(DriveRelative driveRelative) {
        SwerveState swerveState = new SwerveState(this);
        swerveState.driveRelative = driveRelative;
        return swerveState;
    }

    public SwerveState withDriveSpeed(DriveSpeed driveSpeed) {
        SwerveState swerveState = new SwerveState(this);
        swerveState.driveSpeed = driveSpeed;
        return swerveState;
    }

    public SwerveState withLoopMode(LoopMode loopMode) {
        SwerveState swerveState = new SwerveState(this);
        swerveState.loopMode = loopMode;
        return swerveState;
    }

    public SwerveState withRotateAxis(RotateAxis rotateAxis) {
        SwerveState swerveState = new SwerveState(this);
        swerveState.rotateAxis = rotateAxis;
        return swerveState;
    }

    public SwerveState withAimAssist(AimAssist aimAssist) {
        SwerveState swerveState = new SwerveState(this);
        swerveState.aimAssist = aimAssist;
        return swerveState;
    }


    public void updateState(SwerveState newState) {
        this.driveRelative = newState.driveRelative;
        this.driveSpeed = newState.driveSpeed;
        this.loopMode = newState.loopMode;
        this.rotateAxis = newState.rotateAxis;
        this.aimAssist = newState.aimAssist;
    }


    public DriveRelative getDriveMode() {
        return driveRelative;
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

    public AimAssist getAimAssist() {
        return aimAssist;
    }

}
