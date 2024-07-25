package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveConstants;

import java.util.function.Function;
import java.util.function.Supplier;

public class AimAssistUtils {

    /**
     * @param speeds the robot speeds at "field-relative"
     *
     * @return the magnitude of the drive velocities vecotr
     * */
    private static double getDriveMagnitude(ChassisSpeeds speeds) {
        return Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));
    }

    /**
     * @param currentSpeeds current chassis speeds, field relative
     * @param inputSpeeds the speeds that the joysticks command on the robot
     * @param robotPoseSupplier supplier of the robot position
     * @param targetRotationSupplier target angle for the rotation (as a supplier)
     * @param swerveConstants the constants of the swerve
     *
     * @return the aim assisted chassis speeds with only rotation speeds assisted (to look at somewhere)
     *
     * */

    public static ChassisSpeeds getRotationAssistedSpeeds(ChassisSpeeds inputSpeeds, ChassisSpeeds currentSpeeds,
            Supplier<Pose2d> robotPoseSupplier,
            Function<Pose2d, Rotation2d> targetRotationSupplier, SwerveConstants swerveConstants) {

        double driveMagnitude = getDriveMagnitude(currentSpeeds);
        Rotation2d pidVelocity = Rotation2d.fromDegrees(swerveConstants.getRotationDegreesPIDController().calculate(
                robotPoseSupplier.get().getRotation().getDegrees(),
                targetRotationSupplier.apply(robotPoseSupplier.get()).getDegrees()
        ));

        double angularVelocityRads =
                pidVelocity.getRadians() * SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR / (driveMagnitude + SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR);

        double angularVelocityWithJoystick = angularVelocityRads + inputSpeeds.omegaRadiansPerSecond;

        double clampedAngularVelocity = MathUtil.clamp(
                angularVelocityWithJoystick,
                -swerveConstants.getMaxRotationalVelocityPerSecond().getRadians(),
                swerveConstants.getMaxRotationalVelocityPerSecond().getRadians()
        );

        return new ChassisSpeeds(inputSpeeds.vxMetersPerSecond, inputSpeeds.vyMetersPerSecond, clampedAngularVelocity);
    }

}
