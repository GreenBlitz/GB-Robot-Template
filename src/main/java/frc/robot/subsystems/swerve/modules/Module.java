package frc.robot.subsystems.swerve.modules;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.subsystems.swerve.modules.drive.DriveInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.drive.IDrive;
import frc.robot.subsystems.swerve.modules.encoder.IEncoder;
import frc.robot.subsystems.swerve.modules.steer.SteerInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.inputs.ModuleInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.steer.ISteer;
import frc.utils.Conversions;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

public class Module {

    private final ModuleInputsContainer moduleInputsContainer;
    private final ModuleUtils.ModuleName moduleName;
    private final ISteer iSteer;
    private final IDrive iDrive;
    private final IEncoder iEncoder;
    private final ModuleConstants constants;

    private boolean isClosedLoop;
    private SwerveModuleState targetState;
    private Rotation2d startingSteerAngle;

    public Module(ModuleUtils.ModuleName moduleName, ISteer iSteer, IDrive iDrive, IEncoder iEncoder, ModuleConstants constants) {
        this.moduleName = moduleName;
        this.iEncoder = iEncoder;
        this.iSteer = iSteer;
        this.iDrive = iDrive;
        this.constants = constants;
        this.moduleInputsContainer = new ModuleInputsContainer();
        this.targetState = new SwerveModuleState();
        this.isClosedLoop = SwerveState.DEFAULT_DRIVE.getLoopMode().isClosedLoop;
        this.startingSteerAngle = new Rotation2d();
        logStatus();
        resetByEncoder();
    }

    private double toDriveMeters(Rotation2d angle) {
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

    private void updateInputs() {
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
        Rotation2d targetVelocityPerSecond = Conversions.distanceToAngle(
                targetVelocityMetersPerSecond,
                 constants.wheelDiameterMeters()
        );
        Rotation2d optimizedVelocityPerSecond = ModuleUtils.getCoupledAngle(
                targetVelocityPerSecond,
                moduleInputsContainer.getSteerMotorInputs().velocity,
                constants.couplingRatio()
        );
        iDrive.setTargetClosedLoopVelocity(optimizedVelocityPerSecond);
    }

    public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        double voltage = ModuleUtils.velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                moduleInputsContainer.getSteerMotorInputs().velocity,
                constants.couplingRatio(),
                constants.velocityAt12VoltsPerSecond(),
                constants.wheelDiameterMeters(),
                ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        iDrive.runMotorByVoltage(voltage);
    }

}
