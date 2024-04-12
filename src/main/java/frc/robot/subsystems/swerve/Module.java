package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.swerveinterface.IModule;
import frc.robot.subsystems.swerve.swerveinterface.ModuleFactory;
import frc.robot.subsystems.swerve.swerveinterface.ModuleInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.swerve.ModuleUtils.reduceSkew;

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

    public void resetByEncoder() {
        module.resetByEncoder();
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

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(moduleInputs.driveVelocityMetersPerSecond, getCurrentAngle());
    }

    private Rotation2d getCurrentAngle() {
        return Rotation2d.fromDegrees(moduleInputs.steerAngleDegrees);
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(moduleInputs.driveDistanceMeters, getCurrentAngle());
    }

    public SwerveModuleState getTargetState() {
        return targetState;
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
