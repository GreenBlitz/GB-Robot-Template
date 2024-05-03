package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public enum DriveMode {

    NORMAL(
            SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
            SwerveConstants.MAX_ROTATIONAL_SPEED_PER_SECOND
    ),
    SLOW(
            SwerveConstants.MAX_SPEED_METERS_PER_SECOND * SwerveConstants.SLOW_DRIVE_MODE_FACTOR,
            Rotation2d.fromDegrees(SwerveConstants.MAX_ROTATIONAL_SPEED_PER_SECOND.getDegrees() * SwerveConstants.SLOW_DRIVE_MODE_FACTOR)
    );

    public final double MAX_TRANSLATION_SPEED_METERS_PER_SECOND;
    public final Rotation2d MAX_ROTATION_SPEED_PER_SECOND;

    DriveMode(double maxTranslationSpeed, Rotation2d maxRotationSpeed) {
        this.MAX_TRANSLATION_SPEED_METERS_PER_SECOND = maxTranslationSpeed;
        this.MAX_ROTATION_SPEED_PER_SECOND = Rotation2d.fromDegrees(maxRotationSpeed.getDegrees());
    }
}
