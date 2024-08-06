package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveMath;

import java.util.function.Function;
import java.util.function.Supplier;

import static frc.robot.subsystems.swerve.SwerveMath.getDriveMagnitude;

public class AimAssistUtils {

    /**
     * @param currentSpeeds current chassis speeds, field relative
     * @param inputSpeeds the speeds that the joysticks command on the robot
     * @param robotRotationSupplier supplier of the robot rotation
     * @param targetRotationSupplier target angle for the rotation (as a supplier)
     * @param swerveConstants the constants of the swerve
     *
     * @return the aim assisted chassis speeds with only rotation speeds assisted (to look at somewhere)
     *
     * */

    public static ChassisSpeeds getRotationAssistedSpeeds(ChassisSpeeds inputSpeeds,
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

        double angularVelocityRads = getAssistedValueInCompensationToDrive(pidVelocity.getRadians(), currentSpeeds);
        double combinedAngularVelocity = angularVelocityRads + inputSpeeds.omegaRadiansPerSecond;
        Rotation2d clampedAngularVelocityPerSecond = SwerveMath.clampRotationalVelocity(Rotation2d.fromRadians(combinedAngularVelocity),
                swerveConstants.maxRotationalVelocityPerSecond()).getRadians();

        return new ChassisSpeeds(inputSpeeds.vxMetersPerSecond, inputSpeeds.vyMetersPerSecond, clampedAngularVelocity);
    }

    public static double getAssistedValueInCompensationToDrive (double pidGain, ChassisSpeeds driveSpeeds){
        return pidGain * SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR / (getDriveMagnitude(driveSpeeds) + SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR);
    }

}
