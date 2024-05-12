package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import frc.utils.allianceutils.AlliancePose2d;
import frc.utils.allianceutils.AllianceRotation2d;
import frc.utils.allianceutils.AllianceTranslation2d;

public class SwerveMath {

    public static AllianceRotation2d getTargetAngleFromTargetTranslation(AllianceTranslation2d currentPose,
            AllianceTranslation2d targetTranslation) {
        Rotation2d wantedAngle = Rotation2d.fromRadians(
                Math.atan2(
                        targetTranslation.getBlueAllianceTranslation2d().getY() - currentPose.getBlueAllianceTranslation2d().getY(),
                        targetTranslation.getBlueAllianceTranslation2d().getX() - currentPose.getBlueAllianceTranslation2d().getX()
                )
        );
        return AllianceRotation2d.fromBlueAllianceRotation(wantedAngle);
    }

    private ChassisSpeeds getAimAssistedSpeeds(ChassisSpeeds speeds, SwerveState currentState) {
        if (currentState.getAimAssist().equals(AimAssist.NONE)) {
            return speeds;
        }
        Rotation2d pidVelocity = calculateProfiledAngleSpeedToTargetAngle(currentState.getAimAssist().targetAngleSupplier.get());
        //todo - make value have same range like joystick
        //todo - distance factor
        //todo - current robot velocity factor
        speeds.omegaRadiansPerSecond += pidVelocity.getRadians();
        speeds.omegaRadiansPerSecond = MathUtil.clamp(
                speeds.omegaRadiansPerSecond,
                -SwerveConstants.MAX_ROTATIONAL_SPEED_PER_SECOND.getRadians(),
                SwerveConstants.MAX_ROTATIONAL_SPEED_PER_SECOND.getRadians()
        );
        return speeds;
    }

    public static ChassisSpeeds fieldRelativeSpeedsToSelfRelativeSpeeds(
            ChassisSpeeds fieldRelativeSpeeds,
            AlliancePose2d currentPose
    ) {
        Rotation2d currentAngle = currentPose.getAlliancePose().getRotation();
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, currentAngle);
    }

    /**
     * When the robot drives while rotating it skews a bit to the side.
     * This should fix the chassis speeds, so they won't make the robot skew while rotating.
     *
     * @param chassisSpeeds the chassis speeds to fix skewing for
     * @return the fixed speeds
     */
    public static ChassisSpeeds discretize(ChassisSpeeds chassisSpeeds, double timeStepSeconds) {
        return ChassisSpeeds.discretize(chassisSpeeds, timeStepSeconds);
    }

    public static ChassisSpeeds powersToSpeeds(
            double xPower, double yPower, double thetaPower,
            double maxTranslationSpeedMetersPerSecond,
            Rotation2d maxRotationSpeedPerSecond
    ) {
        return new ChassisSpeeds(
                xPower * maxTranslationSpeedMetersPerSecond,
                yPower * maxTranslationSpeedMetersPerSecond,
                Math.pow(thetaPower, 2) * Math.signum(thetaPower) * maxRotationSpeedPerSecond.getRadians()
        );
    }

}
