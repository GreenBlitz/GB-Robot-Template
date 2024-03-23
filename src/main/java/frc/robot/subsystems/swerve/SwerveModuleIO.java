package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.utils.Conversions;
import org.littletonrobotics.junction.Logger;

public class SwerveModuleIO {
    private final SwerveModuleInputsAutoLogged swerveModuleInputs;
    private final String name;
    private boolean driveMotorClosedLoop;
    private SwerveModuleState targetState;

    public SwerveModuleIO(String name) {
        this.name = name;
        this.swerveModuleInputs = new SwerveModuleInputsAutoLogged();
        this.targetState = new SwerveModuleState();
        this.driveMotorClosedLoop = true;
    }

    protected String getLoggingPath() {
        return "Swerve/" + name + "/";
    }

    public void periodic() {
        updateInputs(swerveModuleInputs);
        Logger.processInputs(getLoggingPath(), swerveModuleInputs);
    }


    protected void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
    }

    protected void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
    }

    protected void setTargetAngle(Rotation2d angle) {
    }

    protected void stop() {
    }

    protected void setBrake(boolean brake) {
    }

    protected void updateInputs(SwerveModuleInputsAutoLogged inputs) {
    }


    SwerveModuleState getCurrentState() {
        return new SwerveModuleState(swerveModuleInputs.driveVelocityMetersPerSecond, getCurrentAngle());
    }

    SwerveModuleState getTargetState() {
        return targetState;
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromDegrees(swerveModuleInputs.steerAngleDegrees);
    }

    public void setDriveMotorClosedLoop(boolean closedLoop) {
        driveMotorClosedLoop = closedLoop;
    }


    public void setTargetState(SwerveModuleState targetState) {
        this.targetState = SwerveModuleState.optimize(targetState, getCurrentAngle());
        setTargetAngle(this.targetState.angle);
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

        if (driveMotorClosedLoop)
            setTargetClosedLoopVelocity(targetVelocityMetersPerSecond);
        else
            setTargetOpenLoopVelocity(targetVelocityMetersPerSecond);
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
                swerveModuleInputs.odometryUpdatesDriveDistanceMeters[odometryUpdateIndex],
                Rotation2d.fromDegrees(swerveModuleInputs.odometryUpdatesSteerAngleDegrees[odometryUpdateIndex])
        );
    }
}
