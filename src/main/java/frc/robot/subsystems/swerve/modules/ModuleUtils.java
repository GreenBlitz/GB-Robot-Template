package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.utils.Conversions;

public class ModuleUtils {

    public enum ModuleName {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

    public static String getLoggingPath(ModuleName moduleName) {
        return "Swerve/" + moduleName + "/";
    }

    public static Translation2d getModulePositionRelativeToMiddleOfRobot(ModuleName moduleName) {
        return switch (moduleName) {
            case FRONT_LEFT -> SwerveConstants.FRONT_LEFT_TRANSLATION2D;
            case FRONT_RIGHT -> SwerveConstants.FRONT_RIGHT_TRANSLATION2D;
            case BACK_LEFT -> SwerveConstants.BACK_LEFT_TRANSLATION2D;
            case BACK_RIGHT -> SwerveConstants.BACK_RIGHT_TRANSLATION2D;
        };
    }

    public static double toDriveDistance(Rotation2d revolutions) {
        return Conversions.revolutionsToDistance(revolutions.getRotations(), ModuleConstants.WHEEL_DIAMETER_METERS);
    }

    public static double velocityToOpenLoopVoltage(double velocityMetersPerSecond, double wheelDiameterMeters,
            Rotation2d steerVelocityPerSecond, double couplingRatio, Rotation2d maxSpeedPerSecond,
            double voltageCompensationSaturation
    ) {
        double velocityRevolutionsPerSecond = Conversions.distanceToRevolutions(
                velocityMetersPerSecond,
                wheelDiameterMeters
        );
        double optimizedVelocityRevolutionsPerSecond = removeCouplingFromRevolutions(
                velocityRevolutionsPerSecond,
                steerVelocityPerSecond,
                couplingRatio
        );
        double power = optimizedVelocityRevolutionsPerSecond / maxSpeedPerSecond.getRotations();
        return Conversions.compensatedPowerToVoltage(power, voltageCompensationSaturation);
    }

    /**
     * When the steer motor moves, the drive motor moves as well due to the coupling.
     * This will affect the current position of the drive motor, so we need to remove the coupling from the position.
     *
     * @param drivePosition the position in revolutions
     * @param moduleAngle the angle of the module
     * @return the distance without the coupling
     */
    public static double removeCouplingFromRevolutions(double drivePosition, Rotation2d moduleAngle, double couplingRatio) {
        double coupledAngle = moduleAngle.getRotations() * couplingRatio;
        return drivePosition - coupledAngle;
    }

    /**
     * When changing direction, the module will skew since the angle motor is not at its target angle.
     * This method will counter that by reducing the target velocity according to the angle motor's error cosine.
     *
     * @param targetVelocityMetersPerSecond the target velocity, in meters per second
     * @param targetSteerAngle the target steer angle
     * @return the reduced target velocity in revolutions per second
     */
    public static double reduceSkew(double targetVelocityMetersPerSecond, Rotation2d targetSteerAngle, Rotation2d currentAngle) {
        double closedLoopError = targetSteerAngle.getRadians() - currentAngle.getRadians();
        double cosineScalar = Math.abs(Math.cos(closedLoopError));
        return targetVelocityMetersPerSecond * cosineScalar;
    }

}
