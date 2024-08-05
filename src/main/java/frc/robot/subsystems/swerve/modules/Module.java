package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.LogPaths;
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.subsystems.swerve.modules.drive.DriveInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.drive.IDrive;
import frc.robot.subsystems.swerve.modules.encoder.IEncoder;
import frc.robot.subsystems.swerve.modules.inputs.ModuleInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.steer.ISteer;
import frc.robot.subsystems.swerve.modules.steer.SteerInputsAutoLogged;
import frc.utils.Conversions;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

public class Module {

    private final ModuleInputsContainer moduleInputsContainer;
    private final ModuleUtils.ModulePosition modulePosition;
    private final ISteer iSteer;
    private final IDrive iDrive;
    private final IEncoder iEncoder;
    private final ModuleConstants constants;
    private final String logPath;

    private SwerveModuleState targetState;
    private Rotation2d startingSteerAngle;
    private boolean isClosedLoop;

    public Module(ModuleUtils.ModulePosition modulePosition, ModuleConstants constants, IEncoder iEncoder, ISteer iSteer, IDrive iDrive) {
        this.modulePosition = modulePosition;

        this.constants = constants;
        this.iEncoder = iEncoder;
        this.iSteer = iSteer;
        this.iDrive = iDrive;
        this.moduleInputsContainer = new ModuleInputsContainer();
        this.logPath = this.constants.logPath() + this.modulePosition + "/";

        this.targetState = new SwerveModuleState();
        this.startingSteerAngle = new Rotation2d();
        this.isClosedLoop = SwerveState.DEFAULT_DRIVE.getLoopMode().isClosedLoop;

        logStatus();
        resetByEncoder();
    }

    public double toDriveMeters(Rotation2d angle) {
        return Conversions.angleToDistance(angle, constants.wheelDiameterMeters());
    }

    public void logStatus() {
        updateInputs();
        reportAlerts();
    }

    private void fixDriveInputsCoupling(){
        SteerInputsAutoLogged steerInputs = moduleInputsContainer.getSteerMotorInputs();
        DriveInputsAutoLogged driveInputs = moduleInputsContainer.getDriveMotorInputs();

        driveInputs.angle = ModuleUtils.getUncoupledAngle(driveInputs.angle, steerInputs.angle, constants.couplingRatio());
        driveInputs.velocity = ModuleUtils.getUncoupledAngle(driveInputs.velocity, steerInputs.velocity, constants.couplingRatio());
        driveInputs.acceleration = ModuleUtils.getUncoupledAngle(driveInputs.acceleration, steerInputs.acceleration, constants.couplingRatio());
        for (int i = 0; i < driveInputs.angleOdometrySamples.length; i++){
            Rotation2d steerDelta = Rotation2d.fromRotations(steerInputs.angleOdometrySamples[i].getRotations() - startingSteerAngle.getRotations());
            driveInputs.angleOdometrySamples[i] = ModuleUtils.getUncoupledAngle(driveInputs.angleOdometrySamples[i], steerDelta, constants.couplingRatio());
        }
    }

    public void updateInputs() {
        iEncoder.updateInputs(moduleInputsContainer);
        iSteer.updateInputs(moduleInputsContainer);
        iDrive.updateInputs(moduleInputsContainer);
        fixDriveInputsCoupling();

        DriveInputsAutoLogged driveInputs = moduleInputsContainer.getDriveMotorInputs();
        driveInputs.distanceMeters = toDriveMeters(driveInputs.angle);
        driveInputs.velocityMeters = toDriveMeters(driveInputs.velocity);
        driveInputs.distanceMetersOdometrySamples = Arrays.stream(driveInputs.angleOdometrySamples).mapToDouble(this::toDriveMeters).toArray();


        ModuleInputsAutoLogged moduleInputs = moduleInputsContainer.getModuleInputs();
        moduleInputs.isAtTargetAngle = isAtTargetAngle();
        moduleInputs.isAtTargetVelocity = isAtTargetVelocity();
        moduleInputs.isAtTargetState = moduleInputs.isAtTargetVelocity && moduleInputs.isAtTargetAngle;
        moduleInputs.isClosedLoop = isClosedLoop;

        moduleInputsContainer.processInputs(getModuleLogPath());
    }

    public void reportAlerts() {
        if (!moduleInputsContainer.getEncoderInputs().isConnected) {
            Logger.recordOutput(LogPaths.ALERT_LOG_PATH + logPath + "encoder disconnect", Timer.getFPGATimestamp());
        }
        if (!moduleInputsContainer.getSteerMotorInputs().isConnected) {
            Logger.recordOutput(LogPaths.ALERT_LOG_PATH + logPath + "steer motor disconnect", Timer.getFPGATimestamp());
        }
        if (!moduleInputsContainer.getDriveMotorInputs().isConnected) {
            Logger.recordOutput(LogPaths.ALERT_LOG_PATH + logPath + "drive motor disconnect", Timer.getFPGATimestamp());
        }
    }


    public void setClosedLoop(boolean closedLoop) {
        isClosedLoop = closedLoop;
    }

    public void setBrake(boolean isBrake) {
        iSteer.setBrake(isBrake);
        iDrive.setBrake(isBrake);
    }

    public void resetByEncoder() {
        startingSteerAngle = moduleInputsContainer.getEncoderInputs().angle;
        iSteer.resetToAngle(startingSteerAngle);
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

    public Rotation2d getDriveAngle() {
        return moduleInputsContainer.getDriveMotorInputs().angle;
    }

    public double getDriveVelocityMetersPerSecond() {
        return moduleInputsContainer.getDriveMotorInputs().velocityMeters;
    }

    public Rotation2d getCurrentAngle() {
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


    public void stop() {
        iSteer.stop();
        iDrive.stop();
    }


    public void setDriveVoltage(double voltage) {
        setClosedLoop(false);
        iDrive.setVoltage(voltage);
    }

    public void setSteerVoltage(double voltage) {
        iSteer.setVoltage(voltage);
    }


    public void pointToAngle(Rotation2d angle, boolean optimize) {
        SwerveModuleState moduleState = new SwerveModuleState(0, angle);
        if (optimize) {
            this.targetState.angle = SwerveModuleState.optimize(moduleState, getCurrentAngle()).angle;
        }
        else {
            this.targetState.angle = moduleState.angle;
        }
        iSteer.setTargetAngle(targetState.angle);
    }


    public void setTargetState(SwerveModuleState targetState, boolean isClosedLoop) {
        this.targetState = SwerveModuleState.optimize(targetState, getCurrentAngle());
        iSteer.setTargetAngle(this.targetState.angle);
        setTargetVelocity(this.targetState.speedMetersPerSecond, this.targetState.angle, isClosedLoop);
    }

    public void setTargetVelocity(double targetVelocityMetersPerSecond, Rotation2d targetSteerAngle, boolean isClosedLoop) {
        targetVelocityMetersPerSecond = ModuleUtils.reduceSkew(targetVelocityMetersPerSecond, targetSteerAngle, getCurrentAngle());

        if (isClosedLoop) {
            setTargetClosedLoopVelocity(targetVelocityMetersPerSecond);
        }
        else {
            setTargetOpenLoopVelocity(targetVelocityMetersPerSecond);
        }
    }

    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        setClosedLoop(true);
        Rotation2d targetVelocityPerSecond = Conversions.distanceToAngle(
                targetVelocityMetersPerSecond,
                constants.wheelDiameterMeters()
        );
        Rotation2d coupledVelocityPerSecond = ModuleUtils.getCoupledAngle(
                targetVelocityPerSecond,
                moduleInputsContainer.getSteerMotorInputs().velocity,
                constants.couplingRatio()
        );
        iDrive.setTargetVelocity(coupledVelocityPerSecond);
    }

    public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        setClosedLoop(false);
        double voltage = ModuleUtils.velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                moduleInputsContainer.getSteerMotorInputs().velocity,
                constants.couplingRatio(),
                constants.velocityAt12VoltsPerSecond(),
                constants.wheelDiameterMeters(),
                ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        iDrive.setVoltage(voltage);
    }

}
