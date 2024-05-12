package frc.robot.subsystems.swerve;

// todo - create new class "SwerveMode" which will contain: "DriverDrive", "FollowPath", "Autonomous", "RotateToAngle"

import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import frc.robot.subsystems.swerve.swervestatehelpers.DriveMode;
import frc.robot.subsystems.swerve.swervestatehelpers.DriveSpeed;
import frc.robot.subsystems.swerve.swervestatehelpers.LoopMode;
import frc.robot.subsystems.swerve.swervestatehelpers.RotateAxis;

//todo - add all swerve funcs that depend on this classes into this classes instead of in swerve (if possible)
public class SwerveState {

    public static final DriveMode DEFAULT_DRIVE_MODE = DriveMode.FIELD_RELATIVE;
    public static final DriveSpeed DEFAULT_DRIVE_SPEED = DriveSpeed.NORMAL;
    public static final LoopMode DEFAULT_LOOP_MODE = LoopMode.OPEN;
    public static final RotateAxis DEFAULT_ROTATE_AXIS = RotateAxis.MIDDLE_OF_ROBOT;
    public static final AimAssist DEFAULT_AIM_ASSIST = AimAssist.NONE;


    private DriveMode driveMode;
    private DriveSpeed driveSpeed;
    private LoopMode loopMode;
    private RotateAxis rotateAxis;
    private AimAssist aimAssist;

    public SwerveState() {
        this(DEFAULT_DRIVE_MODE, DEFAULT_DRIVE_SPEED, DEFAULT_LOOP_MODE, DEFAULT_ROTATE_AXIS, DEFAULT_AIM_ASSIST);
    }

    public SwerveState(DriveMode driveMode, DriveSpeed driveSpeed, LoopMode loopMode, RotateAxis rotateAxis, AimAssist aimAssist) {
        this.driveMode = driveMode;
        this.driveSpeed = driveSpeed;
        this.loopMode = loopMode;
        this.rotateAxis = rotateAxis;
        this.aimAssist = aimAssist;
    }

    public SwerveState withDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
        return this;
    }

    public SwerveState withDriveSpeed(DriveSpeed driveSpeed) {
        this.driveSpeed = driveSpeed;
        return this;
    }

    public SwerveState withLoopMode(LoopMode loopMode) {
        this.loopMode = loopMode;
        return this;
    }

    public SwerveState withRotateAxis(RotateAxis rotateAxis) {
        this.rotateAxis = rotateAxis;
        return this;
    }

    public SwerveState withAimAssist(AimAssist aimAssist) {
        this.aimAssist = aimAssist;
        return this;
    }

    public void updateState(SwerveState newState) {
        withDriveMode(newState.driveMode);
        withDriveSpeed(newState.driveSpeed);
        withLoopMode(newState.loopMode);
        withRotateAxis(newState.rotateAxis);
        withAimAssist(newState.aimAssist);
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
