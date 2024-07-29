package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import frc.robot.subsystems.swerve.swervestatehelpers.DriveSpeed;
import frc.utils.cycletime.CycleTimeUtils;

public class SwerveMath {

    public static ChassisSpeeds fieldRelativeToRobotRelativeSpeeds(ChassisSpeeds fieldRelativeSpeeds, Rotation2d allianceRelativeAngle) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, allianceRelativeAngle);
    }

    public static ChassisSpeeds discretize(ChassisSpeeds chassisSpeeds) {
        return ChassisSpeeds.discretize(chassisSpeeds, CycleTimeUtils.getCurrentCycleTime());
    }

    public static ChassisSpeeds powersToSpeeds(double xPower, double yPower, double thetaPower, DriveSpeed driveSpeed, SwerveConstants constants) {
        return new ChassisSpeeds(
                xPower * driveSpeed.translationSpeedFactor * constants.velocityAt12VoltsMetersPerSecond(),
                yPower * driveSpeed.translationSpeedFactor * constants.velocityAt12VoltsMetersPerSecond(),
                thetaPower * driveSpeed.rotationSpeedFactor * constants.maxRotationalVelocityPerSecond().getRadians()
        );
    }

    public static ChassisSpeeds applyDeadband(ChassisSpeeds chassisSpeeds) {
        double newXSpeed = getDeadbandSpeed(chassisSpeeds.vxMetersPerSecond, SwerveConstants.DRIVE_NEUTRAL_DEADBAND);
        double newYSpeed = getDeadbandSpeed(chassisSpeeds.vyMetersPerSecond, SwerveConstants.DRIVE_NEUTRAL_DEADBAND);
        double newOmegaSpeed = getDeadbandSpeed(chassisSpeeds.omegaRadiansPerSecond, SwerveConstants.ROTATION_NEUTRAL_DEADBAND.getRadians());

        return new ChassisSpeeds(newXSpeed, newYSpeed, newOmegaSpeed);
    }

    public static ChassisSpeeds applyAimAssistedRotationVelocity(
            ChassisSpeeds chassisSpeeds,
            Rotation2d currentAngle,
            SwerveState swerveState,
            SwerveConstants constants
    ) {
        if (swerveState.getAimAssist().equals(AimAssist.NONE)) {
            return chassisSpeeds;
        }
        //PID
        Rotation2d pidVelocity = Rotation2d.fromDegrees(constants.rotationDegreesPIDController().calculate(
                currentAngle.getDegrees(),
                swerveState.getAimAssist().targetAngleSupplier.get().getDegrees()
        ));

        //Magnitude Factor
        double driveMagnitude = SwerveMath.getDriveMagnitude(chassisSpeeds);
        double angularVelocityRads =
                pidVelocity.getRadians() * SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR / (driveMagnitude + SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR);

        //Joystick Output
        double angularVelocityWithJoystick = angularVelocityRads + chassisSpeeds.omegaRadiansPerSecond;

        //Clamp
        double clampedAngularVelocity = MathUtil.clamp(
                angularVelocityWithJoystick,
                -constants.maxRotationalVelocityPerSecond().getRadians(),
                constants.maxRotationalVelocityPerSecond().getRadians()
        );

        //todo maybe - make value have stick range (P = MAX_ROT / MAX_ERROR = 10 rads / Math.PI) or clamp between MAX_ROT
        //todo - distance factor
        return new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, clampedAngularVelocity);
    }


    public static boolean isStill(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.vxMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND
                && Math.abs(chassisSpeeds.vyMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND
                && Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= SwerveConstants.ROTATION_NEUTRAL_DEADBAND.getRadians();
    }

    public static double getDriveMagnitude(ChassisSpeeds chassisSpeeds) {
        return Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2) + Math.pow(chassisSpeeds.vyMetersPerSecond, 2));
    }

    public static double getDeadbandSpeed(double speed, double deadband){
        return Math.abs(speed) <= deadband ? 0 : speed;
    }

}
