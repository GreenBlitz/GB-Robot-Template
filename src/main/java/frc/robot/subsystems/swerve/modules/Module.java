package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swerve.modules.moduleinterface.IModule;
import frc.robot.subsystems.swerve.modules.moduleinterface.ModuleFactory;
import frc.robot.subsystems.swerve.modules.moduleinterface.ModuleInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.swerve.modules.ModuleUtils.getAlertLoggingPath;
import static frc.robot.subsystems.swerve.modules.ModuleUtils.reduceSkew;
import static frc.robot.subsystems.swerve.modules.ModuleUtils.toDriveMeters;

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
        updateAllInputs();
    }

    private void updateAllInputs() {
        module.updateInputs(moduleInputs);
        moduleInputs.driveMotorDistanceMeters = toDriveMeters(moduleInputs.driveMotorAngle);
        Logger.processInputs(ModuleUtils.getLoggingPath(moduleName), moduleInputs);
        reportAlertsToLog();
    }

    private void reportAlertsToLog() {
        if (!moduleInputs.allComponentsConnected) {
            Logger.recordOutput(getAlertLoggingPath(moduleName) + "componentDisconnectedAt", Timer.getFPGATimestamp());
        }
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
        return moduleInputs.driveMotorAngle;
    }

    public double getDriveDistanceMeters() {
        return ModuleUtils.toDriveMeters(getDriveDistanceAngle());
    }

    private double getDriveVelocityMetersPerSecond() {
        return ModuleUtils.toDriveMeters(moduleInputs.driveMotorVelocity);
    }

    private Rotation2d getCurrentAngle() {
        return moduleInputs.steerMotorAngle;
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    public boolean isAtTargetState() {
        boolean isAtAngle = isAtAngle(getTargetState().angle);
        boolean isAtVelocity = isAtVelocity(targetState.speedMetersPerSecond);
        return isAtAngle && isAtVelocity;
    }

    public boolean isAtVelocity(double targetSpeedMetersPerSecond) {
        return MathUtil.isNear(
                targetSpeedMetersPerSecond,
                getDriveVelocityMetersPerSecond(),
                ModuleConstants.SPEED_TOLERANCE_METERS_PER_SECOND
        );
    }

    public boolean isAtAngle(Rotation2d targetAngle) {
        boolean isAtAngle = MathUtil.isNear(
                MathUtil.angleModulus(targetAngle.getRadians()),
                MathUtil.angleModulus(getCurrentAngle().getRadians()),
                ModuleConstants.ANGLE_TOLERANCE.getRadians()
        );
        boolean isStopping = moduleInputs.steerMotorVelocity.getRadians() <= ModuleConstants.ANGLE_VELOCITY_TOLERANCE.getRadians();
        return isAtAngle && isStopping;
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
                ModuleUtils.toDriveMeters(moduleInputs.odometryUpdatesDriveDistance[odometryUpdateIndex]),
                moduleInputs.odometryUpdatesSteerAngle[odometryUpdateIndex]
        );
    }

    public int getLastOdometryUpdateIndex() {
        return moduleInputs.odometryUpdatesSteerAngle.length - 1;
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

    public void runSteerMotorByVoltage(double voltage) {
        module.runSteerMotorByVoltage(voltage);
    }

    public void runDriveMotorByVoltage(double voltage) {
        module.runDriveMotorByVoltage(voltage);
    }

}
