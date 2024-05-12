package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.SwerveConstants;

public enum DriveSpeed {

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

    DriveSpeed(double maxTranslationSpeed, Rotation2d maxRotationSpeed) {
        this.maxTranslationSpeedMetersPerSecond = maxTranslationSpeed;
        this.maxRotationSpeedPerSecond = Rotation2d.fromDegrees(maxRotationSpeed.getDegrees());
    }
}
