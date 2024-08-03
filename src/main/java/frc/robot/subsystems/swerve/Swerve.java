package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Field;
import frc.robot.constants.MathConstants;
import frc.robot.poseestimation.observations.OdometryObservation;
import frc.robot.subsystems.swerve.gyro.ISwerveGyro;
import frc.robot.subsystems.swerve.gyro.SwerveGyroConstants;
import frc.robot.subsystems.swerve.gyro.SwerveGyroInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.Modules;
import frc.robot.subsystems.swerve.swervestatehelpers.DriveRelative;
import frc.robot.poseestimation.PoseEstimatorConstants;
import frc.utils.GBSubsystem;
import frc.utils.cycletime.CycleTimeUtils;
import frc.utils.pathplannerutils.PathPlannerUtils;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.Supplier;


public class Swerve extends GBSubsystem {

    public static final Lock ODOMETRY_LOCK = new ReentrantLock();

    private final SwerveCommandsBuilder commandsBuilder;
    private final SwerveGyroInputsAutoLogged gyroInputs;
    private final ISwerveGyro gyro;
    private final Modules modules;
    private final SwerveConstants constants;

    private SwerveState currentState;
    private Supplier<Rotation2d> currentAngleSupplier;

    public Swerve(SwerveConstants constants, Modules modules, ISwerveGyro gyro) {
        super(SwerveConstants.SWERVE_LOG_PATH);
        this.currentState = new SwerveState(SwerveState.DEFAULT_DRIVE);

        this.constants = constants;
        this.modules = modules;
        this.gyro = gyro;

        this.gyroInputs = new SwerveGyroInputsAutoLogged();
        this.currentAngleSupplier = this::getAbsoluteHeading;

        this.commandsBuilder = new SwerveCommandsBuilder(this);
    }

    protected Modules getModules() {
        return modules;
    }

    public SwerveCommandsBuilder getCommandsBuilder() {
        return commandsBuilder;
    }

    @Override
    public String getLogPath() {
        return SwerveConstants.SWERVE_LOG_PATH;
    }


    public void configPathPlanner(Supplier<Pose2d> currentPoseSupplier, Consumer<Pose2d> resetPoseConsumer) {
        PathPlannerUtils.configurePathPlanner(
                currentPoseSupplier,
                resetPoseConsumer,
                this::getRobotRelativeVelocity,
                (speeds) -> driveByState(speeds, SwerveState.DEFAULT_PATH_PLANNER),
                constants.holonomicPathFollowerConfig(),
                () -> !Field.isFieldConventionAlliance(),
                this
        );
    }

    public void setCurrentAngleSupplier(Supplier<Rotation2d> currentAngleSupplier) {
        this.currentAngleSupplier = currentAngleSupplier;
    }

    public void setHeading(Rotation2d heading) {
        gyro.setYaw(heading);
        gyroInputs.gyroYaw = heading;
    }

    protected void resetPIDControllers() {
        constants.xMetersPIDController().reset();
        constants.yMetersPIDController().reset();
        constants.rotationDegreesPIDController().reset();
    }


    @Override
    public void subsystemPeriodic() {
        updateInputs();
        logState();
        logFieldRelativeVelocities();
        logNumberOfOdometrySamples();
    }

    private void updateGyroSimulation() {
        final double yawChangeRadians = getRobotRelativeVelocity().omegaRadiansPerSecond * CycleTimeUtils.getCurrentCycleTime();
        gyroInputs.gyroYaw = Rotation2d.fromRadians(gyroInputs.gyroYaw.getRadians() + yawChangeRadians);
        gyroInputs.yawOdometrySamples = new Rotation2d[]{gyroInputs.gyroYaw};
        gyroInputs.timestampOdometrySamples = new double[]{Timer.getFPGATimestamp()};
    }

    private void reportGyroDisconnect() {
        Logger.recordOutput(SwerveGyroConstants.ALERT_LOG_PATH + "/gyroDisconnectedAt", Timer.getFPGATimestamp());
    }

    private void updateInputs() {
        ODOMETRY_LOCK.lock(); {
            if (gyroInputs.isConnected) {
                reportGyroDisconnect();
                updateGyroSimulation();
            }
            gyro.updateInputs(gyroInputs);
            Logger.processInputs(SwerveConstants.GYRO_LOG_PATH, gyroInputs);

            modules.logStatus();
        } ODOMETRY_LOCK.unlock();
    }

    private void logState() {
        Logger.recordOutput(SwerveConstants.STATE_LOG_PATH + "DriveMode", currentState.getDriveMode());
        Logger.recordOutput(SwerveConstants.STATE_LOG_PATH + "DriveSpeed", currentState.getDriveSpeed());
        Logger.recordOutput(SwerveConstants.STATE_LOG_PATH + "LoopMode", currentState.getLoopMode());
        Logger.recordOutput(SwerveConstants.STATE_LOG_PATH + "RotateAxis", currentState.getRotateAxis());
        Logger.recordOutput(SwerveConstants.STATE_LOG_PATH + "AimAssist", currentState.getAimAssist());
    }

    private void logFieldRelativeVelocities() {
        ChassisSpeeds fieldRelativeSpeeds = getFieldRelativeVelocity();
        Logger.recordOutput(SwerveConstants.VELOCITY_LOG_PATH + "Rotation", fieldRelativeSpeeds.omegaRadiansPerSecond);
        Logger.recordOutput(SwerveConstants.VELOCITY_LOG_PATH + "X", fieldRelativeSpeeds.vxMetersPerSecond);
        Logger.recordOutput(SwerveConstants.VELOCITY_LOG_PATH + "Y", fieldRelativeSpeeds.vyMetersPerSecond);
        Logger.recordOutput(SwerveConstants.VELOCITY_LOG_PATH + "Magnitude", SwerveMath.getDriveMagnitude(fieldRelativeSpeeds));
    }

    private void logNumberOfOdometrySamples() {
        Logger.recordOutput(getLogPath() + "Odometry Samples", getNumberOfOdometrySamples());
    }


    public Rotation2d getAbsoluteHeading() {
        double inputtedHeadingRads = MathUtil.angleModulus(gyroInputs.gyroYaw.getRadians());
        return Rotation2d.fromRadians(inputtedHeadingRads);
    }

    public Rotation2d getRelativeHeading() {
        return Rotation2d.fromDegrees(gyroInputs.gyroYaw.getDegrees());
    }

    public Translation3d getGyroAcceleration() {
        return new Translation3d(gyroInputs.xAcceleration, gyroInputs.yAcceleration, gyroInputs.zAcceleration);
    }


    public int getNumberOfOdometrySamples() {
        return gyroInputs.timestampOdometrySamples.length;
    }

    public OdometryObservation[] getAllOdometryObservations() {
        int odometrySamples = getNumberOfOdometrySamples();
        double[] timestamps = gyroInputs.timestampOdometrySamples;
        Rotation2d[] gyroRotations = gyroInputs.yawOdometrySamples;
        SwerveDriveWheelPositions[] swerveWheelPositions = modules.getAllSwerveWheelPositionSamples();

        OdometryObservation[] odometryObservations = new OdometryObservation[odometrySamples];
        for (int i = 0; i < odometrySamples; i++) {
            odometryObservations[i] = new OdometryObservation(swerveWheelPositions[i], gyroRotations[i], timestamps[i]);
        }

        return odometryObservations;
    }


    public ChassisSpeeds getRobotRelativeVelocity() {
        return SwerveConstants.KINEMATICS.toChassisSpeeds(modules.getModulesStates());
    }

    public ChassisSpeeds getFieldRelativeVelocity() {
        return SwerveMath.robotRelativeToFieldRelativeSpeeds(getRobotRelativeVelocity(), currentAngleSupplier.get());
    }

    public Rotation2d getAllianceRelativeHeading() {
        Rotation2d currentAngle = currentAngleSupplier.get();
        return Field.isFieldConventionAlliance() ? currentAngle : currentAngle.rotateBy(MathConstants.HALF_CIRCLE);
    }

    private ChassisSpeeds getDriveModeRelativeChassisSpeeds(ChassisSpeeds chassisSpeeds, SwerveState swerveState) {
        if (swerveState.getDriveMode() == DriveRelative.ROBOT_RELATIVE) {
            return chassisSpeeds;
        }
        return SwerveMath.fieldRelativeToRobotRelativeSpeeds(chassisSpeeds, getAllianceRelativeHeading());
    }


    /**
     * Runs swerve around itself for WheelRadiusCharacterization
     *
     * @param omegaPerSecond - velocity to run the swerve
     */
    protected void runWheelRadiusCharacterization(Rotation2d omegaPerSecond) {
        driveByState(new ChassisSpeeds(0, 0, omegaPerSecond.getRadians()), SwerveState.DEFAULT_DRIVE);
    }


    protected void pidToPose(Pose2d currentPose, Pose2d targetPose) {
        double xSpeed = constants.xMetersPIDController().calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = constants.yMetersPIDController().calculate(currentPose.getY(), targetPose.getY());
        int direction = Field.isFieldConventionAlliance() ? 1 : -1;
        Rotation2d thetaSpeed = Rotation2d.fromDegrees(constants.rotationDegreesPIDController().calculate(
                currentPose.getRotation().getDegrees(),
                targetPose.getRotation().getDegrees()
        ));

        ChassisSpeeds targetFieldRelativeSpeeds = new ChassisSpeeds(
                xSpeed * direction,
                ySpeed * direction,
                thetaSpeed.getRadians()
        );
        driveByState(targetFieldRelativeSpeeds, SwerveState.DEFAULT_DRIVE);
    }

    protected void rotateToAngle(Rotation2d targetAngle, SwerveState swerveState) {
        ChassisSpeeds targetFieldRelativeSpeeds = new ChassisSpeeds(
                0,
                0,
                Rotation2d.fromDegrees(constants.rotationDegreesPIDController().calculate(
                        currentAngleSupplier.get().getDegrees(),
                        targetAngle.getDegrees()
                )).getRadians()
        );
        driveByState(targetFieldRelativeSpeeds, swerveState);
    }


    protected void driveByState(double xPower, double yPower, double thetaPower, SwerveState swerveState) {
        ChassisSpeeds speedsFromPowers = SwerveMath.powersToSpeeds(xPower, yPower, thetaPower, swerveState.getDriveSpeed(), constants);
        driveByState(speedsFromPowers, swerveState);
    }

    protected void driveByState(ChassisSpeeds chassisSpeeds, SwerveState swerveState) {
        this.currentState = swerveState;

        chassisSpeeds = SwerveMath.applyAimAssistedRotationVelocity(chassisSpeeds, currentAngleSupplier.get(), swerveState, constants);
        if (SwerveMath.isStill(chassisSpeeds)) {
            modules.stop();
            return;
        }

        chassisSpeeds = SwerveMath.applyDeadband(chassisSpeeds);
        chassisSpeeds = getDriveModeRelativeChassisSpeeds(chassisSpeeds, swerveState);
        chassisSpeeds = SwerveMath.discretize(chassisSpeeds);

        applySpeeds(chassisSpeeds, swerveState);
    }

    private void applySpeeds(ChassisSpeeds chassisSpeeds, SwerveState swerveState) {
        SwerveModuleState[] swerveModuleStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(
                chassisSpeeds,
                swerveState.getRotateAxis().getRotateAxis()
        );
        setTargetModuleStates(swerveModuleStates, swerveState.getLoopMode().isClosedLoop);
    }

    private void setTargetModuleStates(SwerveModuleState[] moduleStates, boolean isClosedLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, constants.velocityAt12VoltsMetersPerSecond());
        modules.setTargetModuleStates(moduleStates, isClosedLoop);
    }


    public boolean isAtAngle(Rotation2d targetAngle) {
        double angleDeltaDegrees = Math.abs(targetAngle.minus(currentAngleSupplier.get()).getDegrees());
        boolean isAtAngle = angleDeltaDegrees < PoseEstimatorConstants.ROTATION_TOLERANCE.getDegrees();

        double rotationVelocityRadiansPerSecond = getRobotRelativeVelocity().omegaRadiansPerSecond;
        boolean isStopping = Math.abs(rotationVelocityRadiansPerSecond) < PoseEstimatorConstants.ROTATION_VELOCITY_TOLERANCE.getRadians();

        return isAtAngle && isStopping;
    }

}
