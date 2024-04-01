package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.swerveinterface.ModuleFactory;
import frc.robot.subsystems.swerve.swerveinterface.IModule;
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
    
    public void resetByEncoder(){
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


    public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        module.setTargetOpenLoopVelocity(targetVelocityMetersPerSecond);
    }

    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        module.setTargetClosedLoopVelocity(targetVelocityMetersPerSecond);
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(moduleInputs.driveVelocityMetersPerSecond, getCurrentAngle());
    }

    public SwerveModulePosition getCurrentPosition() {
        return new SwerveModulePosition(moduleInputs.driveDistanceMeters, Rotation2d.fromDegrees(moduleInputs.steerAngleDegrees));
    }

    public SwerveModuleState getTargetState() {
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

    private void setTargetVelocity(double targetVelocityMetersPerSecond, Rotation2d targetSteerAngle) {
        targetVelocityMetersPerSecond = reduceSkew(targetVelocityMetersPerSecond, targetSteerAngle, getCurrentAngle());

        if (driveMotorClosedLoop) {
            setTargetClosedLoopVelocity(targetVelocityMetersPerSecond);
        }
        else {
            setTargetOpenLoopVelocity(targetVelocityMetersPerSecond);
        }
    }


    /**
     * The odometry thread can update itself faster than the main code loop (which is 50 hertz).
     * Instead of using the latest odometry update, the accumulated odometry positions since the last loop to get a more accurate position.
     *
     * @param odometryUpdateIndex the index of the odometry update
     * @return the position of the module at the given odometry update index
     */
    public SwerveModulePosition getOdometryPosition(int odometryUpdateIndex) {
        return new SwerveModulePosition(
                moduleInputs.odometryUpdatesDriveDistanceMeters[odometryUpdateIndex],
                Rotation2d.fromDegrees(moduleInputs.odometryUpdatesSteerAngleDegrees[odometryUpdateIndex])
        );
    }
}
