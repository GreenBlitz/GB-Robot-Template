package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveMath;

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

    public static ChassisSpeeds getRotationAssistedSpeeds(ChassisSpeeds inputSpeeds,
            ChassisSpeeds currentSpeeds,
            Supplier<Pose2d> robotPoseSupplier,
            Function<Pose2d, Rotation2d> targetRotationSupplier,
            SwerveConstants swerveConstants) {

        Rotation2d pidVelocity = Rotation2d.fromDegrees(
                swerveConstants.rotationDegreesPIDController().calculate(
                    robotPoseSupplier.get().getRotation().getDegrees(),
                    targetRotationSupplier.apply(robotPoseSupplier.get()).getDegrees()
                )
        );

        double angularVelocityRads = getAssistedValueInCompensationToDrive(pidVelocity.getRadians(), currentSpeeds);
        double combinedAngularVelocity = angularVelocityRads + inputSpeeds.omegaRadiansPerSecond;
        double clampedAngularVelocity = SwerveMath.clampRotationalVelocity(Rotation2d.fromRadians(combinedAngularVelocity),
                swerveConstants.maxRotationalVelocityPerSecond()).getRadians();

        return new ChassisSpeeds(inputSpeeds.vxMetersPerSecond, inputSpeeds.vyMetersPerSecond, clampedAngularVelocity);
    }

    public static double getAssistedValueInCompensationToDrive (double pidGain, ChassisSpeeds driveSpeeds){
        return pidGain * SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR / (getDriveMagnitude(driveSpeeds) + SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR);
    }

}
