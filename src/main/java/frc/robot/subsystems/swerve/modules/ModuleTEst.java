package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.subsystems.swerve.modules.check.drive.IDrive;
import frc.robot.subsystems.swerve.modules.check.encoder.IEncoder;
import frc.robot.subsystems.swerve.modules.inputs.ModuleInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.inputs.ModuleInputsContainer;
import frc.robot.subsystems.swerve.modules.check.steer.ISteer;
import org.littletonrobotics.junction.Logger;

public class ModuleTEst {

    private final ModuleInputsContainer moduleInputsContainer;
    private final ModuleUtils.ModuleName moduleName;
    private final ISteer iSteer;
    private final IDrive iDrive;
    private final IEncoder iEncoder;

    private boolean isClosedLoop;
    private SwerveModuleState targetState;

    public ModuleTEst(ModuleUtils.ModuleName moduleName, ISteer iSteer, IDrive iDrive, IEncoder iEncoder) {
        this.moduleName = moduleName;
        this.moduleInputsContainer = new ModuleInputsContainer();
        this.targetState = new SwerveModuleState();
        this.isClosedLoop = SwerveState.DEFAULT_DRIVE.getLoopMode().isClosedLoop;
        this.iSteer = iSteer;
        this.iDrive = iDrive;
        this.iEncoder = iEncoder;
        resetByEncoder();
    }

    public void logStatus() {
        updateInputs();
        reportAlerts();
    }

    private void updateInputs() {
        iEncoder.updateInputs(moduleInputsContainer);
        iSteer.updateInputs(moduleInputsContainer);
        iDrive.updateInputs(moduleInputsContainer);

        ModuleInputsAutoLogged moduleInputs = moduleInputsContainer.getModuleInputs();
        moduleInputs.isAtTargetAngle = isAtTargetAngle();
        moduleInputs.isAtTargetVelocity = isAtTargetVelocity();
        moduleInputs.isAtTargetState = moduleInputs.isAtTargetVelocity && moduleInputs.isAtTargetAngle;
        moduleInputs.isClosedLoop = isClosedLoop;

        moduleInputsContainer.processInputs(ModuleUtils.getModuleLogPath(moduleName));
    }

    private void reportAlerts() {
        if (!moduleInputsContainer.getEncoderInputs().isConnected) {
            Logger.recordOutput(ModuleUtils.getModuleAlertLogPath(moduleName) + "encoder disconnect", Timer.getFPGATimestamp());
        }
        if (!moduleInputsContainer.getSteerMotorInputs().isConnected) {
            Logger.recordOutput(ModuleUtils.getModuleAlertLogPath(moduleName) + "steer motor disconnect", Timer.getFPGATimestamp());
        }
        if (!moduleInputsContainer.getDriveMotorInputs().isConnected) {
            Logger.recordOutput(ModuleUtils.getModuleAlertLogPath(moduleName) + "drive motor disconnect", Timer.getFPGATimestamp());
        }
    }


    public void setClosedLoop(boolean closedLoop) {
        isClosedLoop = closedLoop;
    }

    public void stop() {
        iSteer.stop();
        iDrive.stop();
    }

    public void setBrake(boolean isBrake) {
        iSteer.setBrake(isBrake);
        iDrive.setBrake(isBrake);
    }

    public void resetByEncoder() {
        iSteer.resetToAngle(moduleInputsContainer.getEncoderInputs().angle);
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
                moduleInputsContainer.getDriveMotorInputs().distanceMetersOdometrySamples[odometryUpdateIndex],
                moduleInputsContainer.getSteerMotorInputs().angleOdometrySamples[odometryUpdateIndex]
        );
    }

    public int getNumberOfOdometrySamples(){
        return moduleInputsContainer.getDriveMotorInputs().distanceMetersOdometrySamples.length;
    }

    public SwerveModuleState getTargetState() {
        return targetState;
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(getDriveVelocityMetersPerSecond(), getCurrentAngle());
    }

    public Rotation2d getDriveDistanceAngle() {
        return moduleInputsContainer.getDriveMotorInputs().angle;
    }

    private double getDriveVelocityMetersPerSecond() {
        return moduleInputsContainer.getDriveMotorInputs().velocityMeters;
    }

    private Rotation2d getCurrentAngle() {
        return moduleInputsContainer.getSteerMotorInputs().angle;
    }


    public boolean isAtTargetVelocity() {
        return MathUtil.isNear(
                getTargetState().speedMetersPerSecond,
                getDriveVelocityMetersPerSecond(),
                ModuleConstants.SPEED_TOLERANCE_METERS_PER_SECOND
        );
    }

    public boolean isAtTargetAngle() {
        boolean isStopping = moduleInputsContainer.getSteerMotorInputs().velocity.getRadians() <= ModuleConstants.ANGLE_VELOCITY_DEADBAND.getRadians();
        if (!isStopping){
            return false;
        }
        boolean isAtAngle = MathUtil.isNear(
                MathUtil.angleModulus(getTargetState().angle.getRadians()),
                MathUtil.angleModulus(getCurrentAngle().getRadians()),
                ModuleConstants.ANGLE_TOLERANCE.getRadians()
        );
        return isAtAngle;
    }

    public boolean isAtTargetState() {
        return isAtTargetAngle() && isAtTargetVelocity();
    }


    public void pointToAngle(Rotation2d angle, boolean optimize) {
        SwerveModuleState moduleState = new SwerveModuleState(0, angle);
        if (optimize) {
            this.targetState = SwerveModuleState.optimize(moduleState, getCurrentAngle());
        }
        else {
            this.targetState = moduleState;
        }
        iSteer.setTargetAngle(targetState.angle);
    }


    public void runDriveMotorByVoltage(double voltage) {
        iDrive.runMotorByVoltage(voltage);
    }

    public void runSteerMotorByVoltage(double voltage) {
        iSteer.runMotorByVoltage(voltage);
    }


    public void setTargetState(SwerveModuleState targetState, boolean isClosedLoop) {
        setClosedLoop(isClosedLoop);
        this.targetState = SwerveModuleState.optimize(targetState, getCurrentAngle());
        iSteer.setTargetAngle(this.targetState.angle);
        setTargetVelocity(this.targetState.speedMetersPerSecond, this.targetState.angle);
    }

    private void setTargetVelocity(double targetVelocityMetersPerSecond, Rotation2d targetSteerAngle) {
        targetVelocityMetersPerSecond = ModuleUtils.reduceSkew(targetVelocityMetersPerSecond, targetSteerAngle, getCurrentAngle());

        if (isClosedLoop) {
            setTargetClosedLoopVelocity(targetVelocityMetersPerSecond);
        }
        else {
            setTargetOpenLoopVelocity(targetVelocityMetersPerSecond);
        }
    }

    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        iDrive.setTargetClosedLoopVelocity(targetVelocityMetersPerSecond);
    }

    public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        iDrive.setTargetOpenLoopVelocity(targetVelocityMetersPerSecond);
    }

}
