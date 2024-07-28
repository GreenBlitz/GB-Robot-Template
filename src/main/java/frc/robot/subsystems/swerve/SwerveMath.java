package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.swervestatehelpers.DriveSpeed;
import frc.utils.cycletime.CycleTimeUtils;

public class SwerveMath {

    public static ChassisSpeeds applyDeadbandSpeeds(ChassisSpeeds chassisSpeeds) {
        double newXSpeed = chassisSpeeds.vxMetersPerSecond;
        if (Math.abs(chassisSpeeds.vxMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND) {
            newXSpeed = 0;
        }

        double newYSpeed = chassisSpeeds.vyMetersPerSecond;
        if (Math.abs(chassisSpeeds.vyMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND) {
            newYSpeed = 0;
        }

        double newOmegaSpeed = chassisSpeeds.omegaRadiansPerSecond;
        if (Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= SwerveConstants.ROTATION_NEUTRAL_DEADBAND.getRadians()) {
            newOmegaSpeed = 0;
        }

        return new ChassisSpeeds(newXSpeed, newYSpeed, newOmegaSpeed);
    }

    public static ChassisSpeeds fieldRelativeToRobotRelativeSpeeds(ChassisSpeeds fieldRelativeSpeeds, Rotation2d allianceRelativeAngle) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, allianceRelativeAngle);
    }

    public static ChassisSpeeds powersToSpeeds(double xPower, double yPower, double thetaPower, DriveSpeed driveSpeed, SwerveConstants constants) {
        return new ChassisSpeeds(
                xPower * driveSpeed.translationSpeedFactor * constants.velocityAt12VoltsMetersPerSecond(),
                yPower * driveSpeed.translationSpeedFactor * constants.velocityAt12VoltsMetersPerSecond(),
                thetaPower * driveSpeed.rotationSpeedFactor * constants.maxRotationalVelocityPerSecond().getRadians()
        );
    }

    public static ChassisSpeeds discretize(ChassisSpeeds chassisSpeeds) {
        return ChassisSpeeds.discretize(chassisSpeeds, CycleTimeUtils.getCurrentCycleTime());
    }

    public static boolean isStill(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.vxMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND
                && Math.abs(chassisSpeeds.vyMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND
                && Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= SwerveConstants.ROTATION_NEUTRAL_DEADBAND.getRadians();
    }

    public static double getDriveMagnitude(ChassisSpeeds chassisSpeeds) {
        return Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2) + Math.pow(chassisSpeeds.vyMetersPerSecond, 2));
    }

}
