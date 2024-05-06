package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public enum DriveMode {

    //todo - create new class "SwerveState" which will contain:
    // SwerveMode,
    // SwerveRotateAxis,
    // SwerveAimAssist,
    // SwerveDriveMode,
    // SwerveLoopMode (open, close)

    // todo - create new class "SwerveMode" which will contain: "DriverDrive", "Pid/FollowPath", "Autonomous"

    // todo - "SwerveRotateAxis" which will contain: MID_ROBOT and all modules

    // todo - "SwerveAimAssist" which will contain: NON, INTAKE_AIM_ASSIST, SHOOTING_AIM_ASSIST (will contain supplier to wanted angle)

    //todo - add all swerve funcs that depend on this classes into this classes instead of in swerve (if possible)
    NORMAL(
            SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
            SwerveConstants.MAX_ROTATIONAL_SPEED_PER_SECOND
    ),
    SLOW(
            NORMAL.maxTranslationSpeedMetersPerSecond * SwerveConstants.SLOW_DRIVE_MODE_FACTOR,
            Rotation2d.fromDegrees(NORMAL.maxRotationSpeedPerSecond.getDegrees() * SwerveConstants.SLOW_DRIVE_MODE_FACTOR)
    );

    public final double maxTranslationSpeedMetersPerSecond;
    public final Rotation2d maxRotationSpeedPerSecond;

    DriveMode(double maxTranslationSpeed, Rotation2d maxRotationSpeed) {
        this.maxTranslationSpeedMetersPerSecond = maxTranslationSpeed;
        this.maxRotationSpeedPerSecond = Rotation2d.fromDegrees(maxRotationSpeed.getDegrees());
    }
}
