package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.swerve.modules.ModuleUtils.reduceSkew;

public class Module {

    private final ModuleInputsAutoLogged moduleInputs;

    private final ModuleUtils.ModuleName moduleName;

    private final IModule module;

    private boolean driveMotorClosedLoop;

    private SwerveModuleState targetState;

    public Module(ModuleUtils.ModuleName moduleName) {
        this.moduleName = moduleName;
        this.module = ModuleFactory.createModule(moduleName);
        this.moduleInputs = new ModuleInputsAutoLogged();
        this.targetState = new SwerveModuleState();
        this.driveMotorClosedLoop = ModuleConstants.DEFAULT_IS_DRIVE_MOTOR_CLOSED_LOOP;

        resetByEncoder();
    }

    public void periodic() {
        module.updateInputs(moduleInputs);
        Logger.processInputs(ModuleUtils.getLoggingPath(moduleName), moduleInputs);
    }

    public void stop() {
        module.stop();
    }

    public void setBrake(boolean isBrake) {
        module.setBrake(isBrake);
    }

    public void resetByEncoder() {
        module.resetByEncoder();
    }


    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getCurrentAngle());
    }

    public Rotation2d getDriveDistanceAngle() {
        return moduleInputs.driveMotorDistance;
    }

    private double getDriveDistanceMeters() {
        return ModuleUtils.toDriveDistance(getDriveDistanceAngle());
    }

    private double getDriveVelocityMetersPerSecond() {
        return ModuleUtils.toDriveDistance(moduleInputs.driveMotorVelocity);
    }

    private Rotation2d getCurrentAngle() {
        return moduleInputs.steerMotorAngle;
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    /**
     * The odometry thread can update itself faster than the main code loop (which is 50 hertz).
     * Instead of using the latest odometry update, the accumulated odometry positions since the last loop to get a more
     * accurate position.
     *
     * @param odometryUpdateIndex the index of the odometry update
     * @return the position of the module at the given odometry update index
     */
    public SwerveModulePosition getOdometryPosition(int odometryUpdateIndex) {
        return new SwerveModulePosition(
                ModuleUtils.toDriveDistance(Rotation2d.fromDegrees(moduleInputs.odometryUpdatesDriveDistanceDegrees[odometryUpdateIndex])),
                Rotation2d.fromDegrees(moduleInputs.odometryUpdatesSteerAngleDegrees[odometryUpdateIndex])
        );
    }

    public int getLastOdometryUpdateIndex() {
        return moduleInputs.odometryUpdatesSteerAngleDegrees.length - 1;
    }

    public void setTargetState(SwerveModuleState targetState) {
        this.targetState = SwerveModuleState.optimize(targetState, getCurrentAngle());
        module.setTargetAngle(this.targetState.angle);
        setTargetVelocity(this.targetState.speedMetersPerSecond, this.targetState.angle);
    }

    private void setTargetVelocity(double targetVelocityMetersPerSecond, Rotation2d targetSteerAngle) {
        targetVelocityMetersPerSecond = reduceSkew(targetVelocityMetersPerSecond, targetSteerAngle, getCurrentAngle());

        if (driveMotorClosedLoop) {
            setTargetClosedLoopVelocity(targetVelocityMetersPerSecond);
        }
        else {
            setTargetOpenLoopVelocity(targetVelocityMetersPerSecond);
        }
    }

    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        module.setTargetClosedLoopVelocity(targetVelocityMetersPerSecond);
    }

    public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        module.setTargetOpenLoopVelocity(targetVelocityMetersPerSecond);
    }

    public void setDriveMotorClosedLoop(boolean closedLoop) {
        driveMotorClosedLoop = closedLoop;
    }

}
