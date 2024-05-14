package frc.robot.subsystems.swerve;

import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import frc.robot.subsystems.swerve.swervestatehelpers.DriveMode;
import frc.robot.subsystems.swerve.swervestatehelpers.DriveSpeed;
import frc.robot.subsystems.swerve.swervestatehelpers.LoopMode;
import frc.robot.subsystems.swerve.swervestatehelpers.RotateAxis;

public class SwerveState {

    private static final DriveMode DEFAULT_DRIVE_MODE = DriveMode.FIELD_RELATIVE;
    private static final DriveSpeed DEFAULT_DRIVE_SPEED = DriveSpeed.NORMAL;
    private static final LoopMode DEFAULT_LOOP_MODE = LoopMode.OPEN;
    private static final RotateAxis DEFAULT_ROTATE_AXIS = RotateAxis.MIDDLE_OF_ROBOT;
    private static final AimAssist DEFAULT_AIM_ASSIST = AimAssist.NONE;

    public static final SwerveState DEFAULT_PATH_PLANNER = new SwerveState().withDriveMode(DriveMode.SELF_RELATIVE);
    public static final SwerveState DEFAULT_DRIVE = new SwerveState();


    private DriveMode driveMode;
    private DriveSpeed driveSpeed;
    private LoopMode loopMode;
    private RotateAxis rotateAxis;
    private AimAssist aimAssist;

    public SwerveState(SwerveState swerveState) {
        this(swerveState.driveMode, swerveState.driveSpeed, swerveState.loopMode, swerveState.rotateAxis, swerveState.aimAssist);
    }

    private SwerveState() {
        this(DEFAULT_DRIVE_MODE, DEFAULT_DRIVE_SPEED, DEFAULT_LOOP_MODE, DEFAULT_ROTATE_AXIS, DEFAULT_AIM_ASSIST);
    }

    private SwerveState(DriveMode driveMode, DriveSpeed driveSpeed, LoopMode loopMode, RotateAxis rotateAxis, AimAssist aimAssist) {
        this.driveMode = driveMode;
        this.driveSpeed = driveSpeed;
        this.loopMode = loopMode;
        this.rotateAxis = rotateAxis;
        this.aimAssist = aimAssist;
    }


    public SwerveState withDriveMode(DriveMode driveMode) {
        SwerveState swerveState = new SwerveState(this);
        swerveState.driveMode = driveMode;
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
        this.driveMode = newState.driveMode;
        this.driveSpeed = newState.driveSpeed;
        this.loopMode = newState.loopMode;
        this.rotateAxis = newState.rotateAxis;
        this.aimAssist = newState.aimAssist;
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

    public AimAssist getAimAssist() {
        return aimAssist;
    }

}
