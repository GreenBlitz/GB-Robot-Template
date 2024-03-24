package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.mk4iswerve.MK4IModuleConstants;
import frc.robot.subsystems.swerve.swerveinterface.IModule;
import frc.robot.subsystems.swerve.swerveinterface.ModuleFactory;
import frc.robot.subsystems.swerve.swerveinterface.ModuleInputsAutoLogged;
import frc.utils.Conversions;
import org.littletonrobotics.junction.Logger;

public class Module {

    private final ModuleInputsAutoLogged moduleInputs;
    private final ModuleFactory.Module moduleName;
    private boolean driveMotorClosedLoop;
    private IModule module;
    private SwerveModuleState targetState;

    public Module(ModuleFactory.Module moduleName) {
        this.moduleName = moduleName;
        this.module = ModuleFactory.generateModule(moduleName);
        this.moduleInputs = new ModuleInputsAutoLogged();
        this.targetState = new SwerveModuleState();
        this.driveMotorClosedLoop = true;
    }

    protected String getLoggingPath() {
        return "Swerve/" + moduleName + "/";
    }

    public void periodic() {
        module.updateInputs(moduleInputs);
        Logger.processInputs(getLoggingPath(), moduleInputs);
    }

    public void stop() {
        module.stop();
    }

    public void setBrake(boolean isBrake) {
        module.setBrake(isBrake);
    }


    public void setTargetOpenLoopVelocityAndOptimize(double targetVelocityMetersPerSecond) {
        final double voltage = velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                MK4IModuleConstants.WHEEL_DIAMETER_METERS,
                moduleInputs.steerVelocity,
                MK4IModuleConstants.COUPLING_RATIO,
                MK4IModuleConstants.MAX_SPEED_REVOLUTIONS_PER_SECOND,
                MK4IModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        module.setTargetOpenLoopVelocity(voltage);
    }

    public void setTargetClosedLoopVelocityAndOptimize(double targetVelocityMetersPerSecond) {
        final double optimizedVelocityRevolutionsPerSecond = removeCouplingFromRevolutions(
                targetVelocityMetersPerSecond,
                Rotation2d.fromDegrees(moduleInputs.steerVelocity),
                MK4IModuleConstants.COUPLING_RATIO
        );
        module.setTargetClosedLoopVelocity(optimizedVelocityRevolutionsPerSecond);
    }

    SwerveModuleState getCurrentState() {
        return new SwerveModuleState(moduleInputs.driveVelocityMetersPerSecond, getCurrentAngle());
    }

    SwerveModuleState getTargetState() {
        return targetState;
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromDegrees(moduleInputs.steerAngleDegrees);
    }

    public void setDriveMotorClosedLoop(boolean closedLoop) {
        driveMotorClosedLoop = closedLoop;
    }


    public void setTargetState(SwerveModuleState targetState) {
        this.targetState = SwerveModuleState.optimize(targetState, getCurrentAngle());
        module.setTargetAngle(this.targetState.angle);
        setTargetVelocity(this.targetState.speedMetersPerSecond, this.targetState.angle);
    }

    protected double velocityToOpenLoopVoltage(double velocityMetersPerSecond, double wheelDiameterMeters, double steerVelocityRevolutionsPerSecond, double couplingRatio, double maxSpeedRevolutionsPerSecond, double voltageCompensationSaturation) {
        final double velocityRevolutionsPerSecond = Conversions.distanceToRevolutions(velocityMetersPerSecond, wheelDiameterMeters);
        final double optimizedVelocityRevolutionsPerSecond = removeCouplingFromRevolutions(velocityRevolutionsPerSecond, Rotation2d.fromDegrees(steerVelocityRevolutionsPerSecond), couplingRatio);
        final double power = optimizedVelocityRevolutionsPerSecond / maxSpeedRevolutionsPerSecond;
        return Conversions.compensatedPowerToVoltage(power, voltageCompensationSaturation);
    }

    private void setTargetVelocity(double targetVelocityMetersPerSecond, Rotation2d targetSteerAngle) {
        targetVelocityMetersPerSecond = reduceSkew(targetVelocityMetersPerSecond, targetSteerAngle);

        if (driveMotorClosedLoop) {
            setTargetClosedLoopVelocityAndOptimize(targetVelocityMetersPerSecond);
        }
        else {
            setTargetOpenLoopVelocityAndOptimize(targetVelocityMetersPerSecond);
        }
    }


    /**
     * When the steer motor moves, the drive motor moves as well due to the coupling.
     * This will affect the current position of the drive motor, so we need to remove the coupling from the position.
     *
     * @param drivePosition the position in revolutions
     * @param moduleAngle   the angle of the module
     * @return the distance without the coupling
     */
    protected double removeCouplingFromRevolutions(double drivePosition, Rotation2d moduleAngle, double couplingRatio) {
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
    private double reduceSkew(double targetVelocityMetersPerSecond, Rotation2d targetSteerAngle) {
        final double closedLoopError = targetSteerAngle.getRadians() - getCurrentAngle().getRadians();
        final double cosineScalar = Math.abs(Math.cos(closedLoopError));
        return targetVelocityMetersPerSecond * cosineScalar;
    }

    /**
     * The odometry thread can update itself faster than the main code loop (which is 50 hertz).
     * Instead of using the latest odometry update, the accumulated odometry positions since the last loop to get a more accurate position.
     *
     * @param odometryUpdateIndex the index of the odometry update
     * @return the position of the module at the given odometry update index
     */
    SwerveModulePosition getOdometryPosition(int odometryUpdateIndex) {
        return new SwerveModulePosition(
                moduleInputs.odometryUpdatesDriveDistanceMeters[odometryUpdateIndex],
                Rotation2d.fromDegrees(moduleInputs.odometryUpdatesSteerAngleDegrees[odometryUpdateIndex])
        );
    }
}
