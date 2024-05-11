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

    public SwerveState(DriveMode driveMode) {
        this(driveMode, DEFAULT_DRIVE_SPEED, DEFAULT_LOOP_MODE, DEFAULT_ROTATE_AXIS, DEFAULT_AIM_ASSIST);
    }

    public SwerveState(DriveSpeed driveSpeed) {
        this(DEFAULT_DRIVE_MODE, driveSpeed, DEFAULT_LOOP_MODE, DEFAULT_ROTATE_AXIS, DEFAULT_AIM_ASSIST);
    }

    public SwerveState(LoopMode loopMode) {
        this(DEFAULT_DRIVE_MODE, DEFAULT_DRIVE_SPEED, loopMode, DEFAULT_ROTATE_AXIS, DEFAULT_AIM_ASSIST);
    }

    public SwerveState(RotateAxis rotateAxis) {
        this(DEFAULT_DRIVE_MODE, DEFAULT_DRIVE_SPEED, DEFAULT_LOOP_MODE, rotateAxis, DEFAULT_AIM_ASSIST);
    }

    public SwerveState(AimAssist aimAssist) {
        this(DEFAULT_DRIVE_MODE, DEFAULT_DRIVE_SPEED, DEFAULT_LOOP_MODE, DEFAULT_ROTATE_AXIS, aimAssist);
    }

    public SwerveState(DriveMode driveMode, DriveSpeed driveSpeed) {
        this(driveMode, driveSpeed, DEFAULT_LOOP_MODE, DEFAULT_ROTATE_AXIS, DEFAULT_AIM_ASSIST);
    }

    public SwerveState(DriveMode driveMode, LoopMode loopMode) {
        this(driveMode, DEFAULT_DRIVE_SPEED, loopMode, DEFAULT_ROTATE_AXIS, DEFAULT_AIM_ASSIST);
    }

    public SwerveState(DriveMode driveMode, RotateAxis rotateAxis) {
        this(driveMode, DEFAULT_DRIVE_SPEED, DEFAULT_LOOP_MODE, rotateAxis, DEFAULT_AIM_ASSIST);
    }

    public SwerveState(DriveMode driveMode, AimAssist aimAssist) {
        this(driveMode, DEFAULT_DRIVE_SPEED, DEFAULT_LOOP_MODE, DEFAULT_ROTATE_AXIS, aimAssist);
    }

    public SwerveState(DriveSpeed driveSpeed, LoopMode loopMode) {
        this(DEFAULT_DRIVE_MODE, driveSpeed, loopMode, DEFAULT_ROTATE_AXIS, DEFAULT_AIM_ASSIST);
    }

    public SwerveState(DriveSpeed driveSpeed, RotateAxis rotateAxis) {
        this(DEFAULT_DRIVE_MODE, driveSpeed, DEFAULT_LOOP_MODE, rotateAxis, DEFAULT_AIM_ASSIST);
    }

    public SwerveState(DriveSpeed driveSpeed, AimAssist aimAssist) {
        this(DEFAULT_DRIVE_MODE, driveSpeed, DEFAULT_LOOP_MODE, DEFAULT_ROTATE_AXIS, aimAssist);
    }

    public SwerveState(LoopMode loopMode, RotateAxis rotateAxis) {
        this(DEFAULT_DRIVE_MODE, DEFAULT_DRIVE_SPEED, loopMode, rotateAxis, DEFAULT_AIM_ASSIST);
    }

    public SwerveState(LoopMode loopMode, AimAssist aimAssist) {
        this(DEFAULT_DRIVE_MODE, DEFAULT_DRIVE_SPEED, loopMode, DEFAULT_ROTATE_AXIS, aimAssist);
    }

    public SwerveState(RotateAxis rotateAxis, AimAssist aimAssist) {
        this(DEFAULT_DRIVE_MODE, DEFAULT_DRIVE_SPEED, DEFAULT_LOOP_MODE, rotateAxis, aimAssist);
    }

    public SwerveState(DriveMode driveMode, DriveSpeed driveSpeed, LoopMode loopMode) {
        this(driveMode, driveSpeed, loopMode, DEFAULT_ROTATE_AXIS, DEFAULT_AIM_ASSIST);
    }

    public SwerveState(DriveMode driveMode, DriveSpeed driveSpeed, RotateAxis rotateAxis) {
        this(driveMode, driveSpeed, DEFAULT_LOOP_MODE, rotateAxis, DEFAULT_AIM_ASSIST);
    }

    public SwerveState(DriveMode driveMode, DriveSpeed driveSpeed, AimAssist aimAssist) {
        this(driveMode, driveSpeed, DEFAULT_LOOP_MODE, DEFAULT_ROTATE_AXIS, aimAssist);
    }

    public SwerveState(DriveMode driveMode, LoopMode loopMode, RotateAxis rotateAxis) {
        this(driveMode, DEFAULT_DRIVE_SPEED, loopMode, rotateAxis, DEFAULT_AIM_ASSIST);
    }

    public SwerveState(DriveMode driveMode, LoopMode loopMode, AimAssist aimAssist) {
        this(driveMode, DEFAULT_DRIVE_SPEED, loopMode, DEFAULT_ROTATE_AXIS, aimAssist);
    }

    public SwerveState(DriveMode driveMode, RotateAxis rotateAxis, AimAssist aimAssist) {
        this(driveMode, DEFAULT_DRIVE_SPEED, DEFAULT_LOOP_MODE, rotateAxis, aimAssist);
    }

    public SwerveState(DriveSpeed driveSpeed, LoopMode loopMode, RotateAxis rotateAxis) {
        this(DEFAULT_DRIVE_MODE, driveSpeed, loopMode, rotateAxis, DEFAULT_AIM_ASSIST);
    }

    public SwerveState(DriveSpeed driveSpeed, LoopMode loopMode, AimAssist aimAssist) {
        this(DEFAULT_DRIVE_MODE, driveSpeed, loopMode, DEFAULT_ROTATE_AXIS, aimAssist);
    }

    public SwerveState(DriveSpeed driveSpeed, RotateAxis rotateAxis, AimAssist aimAssist) {
        this(DEFAULT_DRIVE_MODE, driveSpeed, DEFAULT_LOOP_MODE, rotateAxis, aimAssist);
    }

    public SwerveState(LoopMode loopMode, RotateAxis rotateAxis, AimAssist aimAssist) {
        this(DEFAULT_DRIVE_MODE, DEFAULT_DRIVE_SPEED, loopMode, rotateAxis, aimAssist);
    }

    public SwerveState(DriveMode driveMode, DriveSpeed driveSpeed, LoopMode loopMode, RotateAxis rotateAxis) {
        this(driveMode, driveSpeed, loopMode, rotateAxis, DEFAULT_AIM_ASSIST);
    }

    public SwerveState(DriveMode driveMode, DriveSpeed driveSpeed, LoopMode loopMode, AimAssist aimAssist) {
        this(driveMode, driveSpeed, loopMode, DEFAULT_ROTATE_AXIS, aimAssist);
    }

    public SwerveState(DriveMode driveMode, DriveSpeed driveSpeed, RotateAxis rotateAxis, AimAssist aimAssist) {
        this(driveMode, driveSpeed, DEFAULT_LOOP_MODE, rotateAxis, aimAssist);
    }

    public SwerveState(DriveMode driveMode, LoopMode loopMode, RotateAxis rotateAxis, AimAssist aimAssist) {
        this(driveMode, DEFAULT_DRIVE_SPEED, loopMode, rotateAxis, aimAssist);
    }

    public SwerveState(DriveSpeed driveSpeed, LoopMode loopMode, RotateAxis rotateAxis, AimAssist aimAssist) {
        this(DEFAULT_DRIVE_MODE, driveSpeed, loopMode, rotateAxis, aimAssist);
    }

    public SwerveState(DriveMode driveMode, DriveSpeed driveSpeed, LoopMode loopMode, RotateAxis rotateAxis, AimAssist aimAssist) {
        this.driveMode = driveMode;
        this.driveSpeed = driveSpeed;
        this.loopMode = loopMode;
        this.rotateAxis = rotateAxis;
        this.aimAssist = aimAssist;
    }

    public void updateState(SwerveState newState) {
        driveMode = newState.driveMode;
        driveSpeed = newState.driveSpeed;
        loopMode = newState.loopMode;
        rotateAxis = newState.rotateAxis;
        aimAssist = newState.aimAssist;
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
