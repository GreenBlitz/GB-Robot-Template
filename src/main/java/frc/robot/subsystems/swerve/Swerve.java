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
import frc.robot.Robot;
import frc.robot.constants.MathConstants;
import frc.robot.poseestimation.observations.OdometryObservation;
import frc.robot.subsystems.swerve.gyro.ISwerveGyro;
import frc.robot.subsystems.swerve.gyro.SwerveGyroConstants;
import frc.robot.subsystems.swerve.gyro.SwerveGyroInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.Modules;
import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import frc.robot.subsystems.swerve.swervestatehelpers.DriveRelative;
import frc.robot.subsystems.swerve.swervestatehelpers.DriveSpeed;
import frc.robot.superstructers.poseestimator.PoseEstimatorConstants;
import frc.utils.DriverStationUtils;
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
    private final SwerveState currentState;
    private final SwerveConstants constants;
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

    public void buildPathPlannerForAuto(Supplier<Pose2d> currentPoseSupplier, Consumer<Pose2d> resetPoseConsumer) {
        PathPlannerUtils.configurePathPlanner(
                currentPoseSupplier,
                resetPoseConsumer, // todo - maybe cancel and base vision
                this::getRobotRelativeVelocity,
                (speeds) -> driveByState(speeds, SwerveState.DEFAULT_PATH_PLANNER), // todo: Will not change loop mode!!!
                constants.holonomicPathFollowerConfig(),
                DriverStationUtils::isRedAlliance,
                this
        );
    }

    @Override
    public String getLogPath() {
        return SwerveConstants.SWERVE_LOG_PATH;
    }

    @Override
    public void subsystemPeriodic() {
        logState();
        logFieldRelativeVelocities();
        logNumberOfOdometrySamples();
    }

    private void logState() {
        Logger.recordOutput(SwerveConstants.SWERVE_STATE_LOG_PATH + "DriveMode", currentState.getDriveMode());
        Logger.recordOutput(SwerveConstants.SWERVE_STATE_LOG_PATH + "DriveSpeed", currentState.getDriveSpeed());
        Logger.recordOutput(SwerveConstants.SWERVE_STATE_LOG_PATH + "LoopMode", currentState.getLoopMode());
        Logger.recordOutput(SwerveConstants.SWERVE_STATE_LOG_PATH + "RotateAxis", currentState.getRotateAxis());
        Logger.recordOutput(SwerveConstants.SWERVE_STATE_LOG_PATH + "AimAssist", currentState.getAimAssist());
    }

    private void logFieldRelativeVelocities() {
        ChassisSpeeds fieldRelativeSpeeds = getFieldRelativeVelocity();
        Logger.recordOutput(SwerveConstants.SWERVE_VELOCITY_LOG_PATH + "Rotation", fieldRelativeSpeeds.omegaRadiansPerSecond);
        Logger.recordOutput(SwerveConstants.SWERVE_VELOCITY_LOG_PATH + "X", fieldRelativeSpeeds.vxMetersPerSecond);
        Logger.recordOutput(SwerveConstants.SWERVE_VELOCITY_LOG_PATH + "Y", fieldRelativeSpeeds.vyMetersPerSecond);
        Logger.recordOutput(SwerveConstants.SWERVE_VELOCITY_LOG_PATH + "Magnitude", getDriveMagnitude(fieldRelativeSpeeds));
    }

    private void logNumberOfOdometrySamples() {
        Logger.recordOutput(getLogPath() + "Odometry Samples", getNumberOfOdometrySamples());
    }

    private void updateGyroSimulation() {
        final double yawChangeRadians = getRobotRelativeVelocity().omegaRadiansPerSecond * CycleTimeUtils.getCurrentCycleTime();
        gyroInputs.gyroYaw = Rotation2d.fromRadians(gyroInputs.gyroYaw.getRadians() + yawChangeRadians);
        gyroInputs.yawOdometrySamples = new Rotation2d[]{gyroInputs.gyroYaw};
        gyroInputs.timestampOdometrySamples = new double[]{Timer.getFPGATimestamp()};
    }

    public void updateInputs() {
        ODOMETRY_LOCK.lock(); {
            if (Robot.ROBOT_TYPE.isSimulation()) {
                updateGyroSimulation();
            }
            gyro.updateInputs(gyroInputs);
            Logger.processInputs(SwerveGyroConstants.LOG_PATH, gyroInputs);

            modules.logStatus();
        } ODOMETRY_LOCK.unlock();
    } // todo: fix


    protected void initializeDrive(SwerveState updatedState) {
        currentState.update(updatedState);
        modules.setClosedLoopForModules(currentState.getLoopMode());;
        constants.translationMetersPIDController().reset();
        constants.rotationDegreesPIDController().reset();
    }

    public void setCurrentAngleSupplier(Supplier<Rotation2d> currentAngleSupplier) {
        this.currentAngleSupplier = currentAngleSupplier;
    }

    public void setHeading(Rotation2d heading) {
        gyro.setHeading(heading);
        gyroInputs.gyroYaw = heading;
    }


    public Rotation2d getAbsoluteHeading() {
        double inputtedHeadingRads = MathUtil.angleModulus(gyroInputs.gyroYaw.getRadians());
        return Rotation2d.fromRadians(inputtedHeadingRads);
    }

    public Rotation2d getRelativeHeading() {
        return Rotation2d.fromDegrees(gyroInputs.gyroYaw.getDegrees());
    }

    public Translation3d getGyroAcceleration() {
        return new Translation3d(gyroInputs.accelerationX, gyroInputs.accelerationY, gyroInputs.accelerationZ);
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
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeVelocity(), currentAngleSupplier.get());
    }

    private ChassisSpeeds getDriveModeRelativeChassisSpeeds(ChassisSpeeds chassisSpeeds, SwerveState swerveState) {
        if (swerveState.getDriveMode() == DriveRelative.ROBOT_RELATIVE) {
            return chassisSpeeds;
        }
        else {
            return fieldRelativeToRobotRelativeSpeeds(chassisSpeeds, getAllianceRelativeAngle());
        }
    }

    public Rotation2d getAllianceRelativeAngle() {
        Rotation2d currentAngle = currentAngleSupplier.get();
        return DriverStationUtils.isRedAlliance() ? currentAngle.rotateBy(MathConstants.HALF_CIRCLE) : currentAngle;
    }


    /**
     * Runs swerve around itself for WheelRadiusCharacterization
     *
     * @param omegaPerSecond - velocity to run the swerve
     */
    protected void runWheelRadiusCharacterization(Rotation2d omegaPerSecond) {
        driveByState(new ChassisSpeeds(0, 0, omegaPerSecond.getRadians()));
    }


    protected void pidToPose(Pose2d currentPose, Pose2d targetPose) {
        double xSpeed = constants.translationMetersPIDController().calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = constants.translationMetersPIDController().calculate(currentPose.getY(), targetPose.getY());
        int direction = DriverStationUtils.isBlueAlliance() ? 1 : -1;
        Rotation2d thetaSpeed = calculateAngleSpeedToTargetAngle(currentAngleSupplier.get(), targetPose.getRotation());

        ChassisSpeeds targetFieldRelativeSpeeds = new ChassisSpeeds(
                xSpeed * direction,
                ySpeed * direction,
                thetaSpeed.getRadians()
        );
        driveByState(targetFieldRelativeSpeeds);
    }

    protected void rotateToAngle(Rotation2d targetAngle) {
        ChassisSpeeds targetFieldRelativeSpeeds = new ChassisSpeeds(
                0,
                0,
                calculateAngleSpeedToTargetAngle(currentAngleSupplier.get(), targetAngle).getRadians()
        );
        driveByState(targetFieldRelativeSpeeds);
    }

    private Rotation2d calculateAngleSpeedToTargetAngle(Rotation2d currentAngle, Rotation2d targetAngle) {
        return Rotation2d.fromDegrees(constants.rotationDegreesPIDController().calculate(
                currentAngle.getDegrees(),
                targetAngle.getDegrees()
        ));
    }


    protected void driveByState(double xPower, double yPower, double thetaPower) {
        driveByState(powersToSpeeds(xPower, yPower, thetaPower, currentState.getDriveSpeed(), constants));
    }

    private void driveByState(ChassisSpeeds chassisSpeeds) {
        driveByState(chassisSpeeds, currentState);
    }

    protected void driveByState(ChassisSpeeds chassisSpeeds, SwerveState swerveState) {
        chassisSpeeds = applyAimAssistedRotationVelocity(chassisSpeeds, currentAngleSupplier.get(), swerveState);

        if (isStill(chassisSpeeds)) {
            modules.stop();
            return;
        }

        chassisSpeeds = applyDeadbandSpeeds(chassisSpeeds);
        chassisSpeeds = getDriveModeRelativeChassisSpeeds(chassisSpeeds, swerveState);
        chassisSpeeds = discretize(chassisSpeeds);

        applySpeeds(chassisSpeeds, swerveState);
    }

    private void applySpeeds(ChassisSpeeds chassisSpeeds, SwerveState swerveState) {
        SwerveModuleState[] swerveModuleStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(
                chassisSpeeds,
                swerveState.getRotateAxis().getRotateAxis()
        );
        setTargetModuleStates(swerveModuleStates);
    }

    private void setTargetModuleStates(SwerveModuleState[] moduleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, constants.velocityAt12VoltsMetersPerSecond());
        modules.setTargetModuleStates(moduleStates);
    }


    public boolean isAtAngle(Rotation2d targetAngle) {
        double angleDifferenceDeg = Math.abs(targetAngle.minus(currentAngleSupplier.get()).getDegrees());
        boolean isAtAngle = angleDifferenceDeg < PoseEstimatorConstants.ROTATION_TOLERANCE.getDegrees();

        double currentRotationVelocityRadians = getRobotRelativeVelocity().omegaRadiansPerSecond;
        boolean isStopping = Math.abs(currentRotationVelocityRadians) < PoseEstimatorConstants.ROTATION_VELOCITY_TOLERANCE.getRadians();

        return isAtAngle && isStopping;
    }


    //todo: make shorter
    private ChassisSpeeds applyAimAssistedRotationVelocity(ChassisSpeeds chassisSpeeds, Rotation2d currentAngle, SwerveState swerveState) {
        if (swerveState.getAimAssist().equals(AimAssist.NONE)) {
            return chassisSpeeds;
        }
        //PID
        Rotation2d pidVelocity = calculateAngleSpeedToTargetAngle(currentAngle, swerveState.getAimAssist().targetAngleSupplier.get());

        //Magnitude Factor
        double driveMagnitude = getDriveMagnitude(chassisSpeeds);
        double angularVelocityRads =
                pidVelocity.getRadians() * SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR / (driveMagnitude + SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR);

        //Joystick Output
        double angularVelocityWithJoystick = angularVelocityRads + chassisSpeeds.omegaRadiansPerSecond;

        //Clamp
        double clampedAngularVelocity = MathUtil.clamp(
                angularVelocityWithJoystick,
                -constants.maxRotationalVelocityPerSecond().getRadians(),
                constants.maxRotationalVelocityPerSecond().getRadians()
        );

        //todo maybe - make value have stick range (P = MAX_ROT / MAX_ERROR = 10 rads / Math.PI) or clamp between MAX_ROT
        //todo - distance factor
        return new ChassisSpeeds(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, clampedAngularVelocity);
    }

    private static ChassisSpeeds applyDeadbandSpeeds(ChassisSpeeds chassisSpeeds) {
        double newXSpeed = chassisSpeeds.vxMetersPerSecond;
        if (Math.abs(chassisSpeeds.vxMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND) {
            newXSpeed = 0;
        }

        double newYSpeed = chassisSpeeds.vyMetersPerSecond;
        if (Math.abs(chassisSpeeds.vyMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND) {
            newYSpeed = 0;
        }

        double newOmegaSpeed = chassisSpeeds.omegaRadiansPerSecond;
        if (Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= SwerveConstants.ROTATION_NEUTRAL_DEADBAND.getRadians()) {
            newOmegaSpeed = 0;
        }

        return new ChassisSpeeds(newXSpeed, newYSpeed, newOmegaSpeed);
    }

    private static ChassisSpeeds fieldRelativeToRobotRelativeSpeeds(ChassisSpeeds fieldRelativeSpeeds, Rotation2d allianceRelativeAngle) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, allianceRelativeAngle);
    }

    private static ChassisSpeeds powersToSpeeds(double xPower, double yPower, double thetaPower, DriveSpeed driveSpeed, SwerveConstants constants) {
        return new ChassisSpeeds(
                xPower * driveSpeed.translationSpeedFactor * constants.velocityAt12VoltsMetersPerSecond(),
                yPower * driveSpeed.translationSpeedFactor * constants.velocityAt12VoltsMetersPerSecond(),
                thetaPower * driveSpeed.rotationSpeedFactor * constants.maxRotationalVelocityPerSecond().getRadians()
        );
    }

    private static ChassisSpeeds discretize(ChassisSpeeds chassisSpeeds) {
        return ChassisSpeeds.discretize(chassisSpeeds, CycleTimeUtils.getCurrentCycleTime());
    }

    private static boolean isStill(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.vxMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND
                && Math.abs(chassisSpeeds.vyMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND
                && Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= SwerveConstants.ROTATION_NEUTRAL_DEADBAND.getRadians();
    }

    private static double getDriveMagnitude(ChassisSpeeds chassisSpeeds) {
        return Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2) + Math.pow(chassisSpeeds.vyMetersPerSecond, 2));
    }

}
