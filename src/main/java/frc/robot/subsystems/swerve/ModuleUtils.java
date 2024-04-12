package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.Conversions;

public class ModuleUtils {

    public static String getLoggingPath(ModuleName moduleName) {
        return "Swerve/" + moduleName + "/";
    }

    public static double toDriveDistance(double revolutions) {
        return Conversions.revolutionsToDistance(revolutions, ModuleConstants.WHEEL_DIAMETER_METERS);
    }

    public static double velocityToOpenLoopVoltage(
            double velocityMetersPerSecond,
            double wheelDiameterMeters,
            double steerVelocityRevolutionsPerSecond,
            double couplingRatio,
            double maxSpeedRevolutionsPerSecond,
            double voltageCompensationSaturation
    ) {
        final double velocityRevolutionsPerSecond = Conversions.distanceToRevolutions(velocityMetersPerSecond, wheelDiameterMeters);
        final double optimizedVelocityRevolutionsPerSecond = removeCouplingFromRevolutions(velocityRevolutionsPerSecond,
                Rotation2d.fromDegrees(steerVelocityRevolutionsPerSecond),
                couplingRatio
        );
        final double power = optimizedVelocityRevolutionsPerSecond / maxSpeedRevolutionsPerSecond;
        return Conversions.compensatedPowerToVoltage(power, voltageCompensationSaturation);
    }

    /**
     * When the steer motor moves, the drive motor moves as well due to the coupling.
     * This will affect the current position of the drive motor, so we need to remove the coupling from the position.
     *
     * @param drivePosition the position in revolutions
     * @param moduleAngle   the angle of the module
     * @return the distance without the coupling
     */
    public static double removeCouplingFromRevolutions(
            double drivePosition, Rotation2d moduleAngle, double couplingRatio
    ) {
        final double coupledAngle = moduleAngle.getRotations() * couplingRatio;
        return drivePosition - coupledAngle;
    }

    /**
     * When changing direction, the module will skew since the angle motor is not at its target angle.
     * This method will counter that by reducing the target velocity according to the angle motor's error cosine.
     *
     * @param targetVelocityMetersPerSecond the target velocity, in meters per second
     * @param targetSteerAngle              the target steer angle
     * @return the reduced target velocity in revolutions per second
     */
    public static double reduceSkew(
            double targetVelocityMetersPerSecond, Rotation2d targetSteerAngle, Rotation2d currentAngle
    ) {
        final double closedLoopError = targetSteerAngle.getRadians() - currentAngle.getRadians();
        final double cosineScalar = Math.abs(Math.cos(closedLoopError));
        return targetVelocityMetersPerSecond * cosineScalar;
    }

    public enum ModuleName {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

}
