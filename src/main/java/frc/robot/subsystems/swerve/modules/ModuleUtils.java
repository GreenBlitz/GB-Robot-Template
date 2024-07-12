package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.Conversions;

public class ModuleUtils {

    public enum ModuleName {

        FRONT_LEFT(0),
        FRONT_RIGHT(1),
        BACK_LEFT(2),
        BACK_RIGHT(3);

        private final int index;

        ModuleName(int index) {
            this.index = index;
        }

        public int getIndex() {
            return index;
        }

    }

    public static String getLoggingPath(ModuleName moduleName) {
        return ModuleConstants.LOG_PATH + moduleName + "/";
    }

    public static String getAlertLoggingPath(ModuleName moduleName) {
        return ModuleConstants.ALERT_LOG_PATH + moduleName + "/";
    }


    public static double velocityToOpenLoopVoltage(
            double velocityMetersPerSecond, Rotation2d steerVelocityPerSecond,
            double couplingRatio, Rotation2d maxSpeedPerSecond,
            double wheelDiameterMeters, double voltageCompensationSaturation
    ) {
        Rotation2d velocityPerSecond = Conversions.distanceToAngle(velocityMetersPerSecond, wheelDiameterMeters);
        double optimizedVelocityRevolutionsPerSecond = removeCouplingFromRevolutions(
                velocityPerSecond,
                steerVelocityPerSecond,
                couplingRatio
        );
        double power = optimizedVelocityRevolutionsPerSecond / maxSpeedPerSecond.getRotations();
        return Conversions.compensatedPowerToVoltage(power, voltageCompensationSaturation);
    }

    /**
     * When the steer motor moves, the drive motor moves as well due to the coupling.
     * This will affect the current position of the drive motor, so we need to remove the coupling from the
     * velocity or the position.
     *
     * @param drivePosition the position or velocity
     * @param moduleAngle the angle or velocity in angle of the module
     * @return the distance or velocity without the coupling
     */
    public static double removeCouplingFromRevolutions(Rotation2d drivePosition, Rotation2d moduleAngle, double couplingRatio) {
        double coupledAngle = moduleAngle.getRotations() * couplingRatio;
        return drivePosition.getRotations() - coupledAngle;
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
