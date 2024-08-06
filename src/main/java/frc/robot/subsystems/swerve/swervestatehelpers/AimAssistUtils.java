package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveMath;

import java.util.function.Supplier;

import static frc.robot.subsystems.swerve.SwerveMath.getDriveMagnitude;

public class AimAssistUtils {

    public static ChassisSpeeds getRotationAssistedSpeeds(ChassisSpeeds speeds,
            ChassisSpeeds currentSpeeds,
            Supplier<Rotation2d> robotRotationSupplier,
            Supplier<Rotation2d> targetRotationSupplier,
            SwerveConstants swerveConstants) {

        Rotation2d pidVelocity = Rotation2d.fromDegrees(
                swerveConstants.rotationDegreesPIDController().calculate(
                    robotRotationSupplier.get().getDegrees(),
                    targetRotationSupplier.get().getDegrees()
                )
        );

        double angularVelocityRadians = applyMagnitudeCompensation(pidVelocity, currentSpeeds);
        double combinedAngularVelocityRadians = angularVelocityRadians + speeds.omegaRadiansPerSecond;
        Rotation2d clampedAngularVelocityPerSecond = SwerveMath.clampRotationalVelocity(Rotation2d.fromRadians(combinedAngularVelocityRadians),
                swerveConstants.maxRotationalVelocityPerSecond());

        return new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, clampedAngularVelocityPerSecond.getRadians());
    }

    public static double applyMagnitudeCompensation(Rotation2d pidGain, ChassisSpeeds driveSpeeds){
        return pidGain.getRadians() * SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR / (getDriveMagnitude(driveSpeeds) + SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR);
    }

}
