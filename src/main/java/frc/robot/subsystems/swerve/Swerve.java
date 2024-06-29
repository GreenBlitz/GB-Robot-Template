package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.MathConstants;
import frc.robot.subsystems.swerve.modules.Module;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.swervegyro.SwerveGyroConstants;
import frc.robot.subsystems.swerve.swervegyro.swervegyrointerface.ISwerveGyro;
import frc.robot.subsystems.swerve.swervegyro.swervegyrointerface.SwerveGyroFactory;
import frc.robot.subsystems.swerve.swervegyro.swervegyrointerface.SwerveGyroInputsAutoLogged;
import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import frc.robot.subsystems.swerve.swervestatehelpers.DriveMode;
import frc.utils.DriverStationUtils;
import frc.utils.GBSubsystem;
import frc.utils.cycletimeutils.CycleTimeUtils;
import frc.utils.mirrorutils.MirrorablePose2d;
import frc.utils.mirrorutils.MirrorableRotation2d;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;

public class Swerve extends GBSubsystem {

    public static final Lock ODOMETRY_LOCK = new ReentrantLock();

    private final SwerveGyroInputsAutoLogged gyroInputs;

    private final ISwerveGyro gyro;

    private final Module[] modules;

    private final SwerveState currentState;


    public Swerve() {
        setName("Swerve");
        this.currentState = new SwerveState(SwerveState.DEFAULT_DRIVE);
        this.modules = getModules();
        this.gyro = SwerveGyroFactory.createSwerveGyro();
        this.gyroInputs = new SwerveGyroInputsAutoLogged();
    }

    private Module[] getModules() {
        return new Module[]{
                new Module(ModuleUtils.ModuleName.FRONT_LEFT),
                new Module(ModuleUtils.ModuleName.FRONT_RIGHT),
                new Module(ModuleUtils.ModuleName.BACK_LEFT),
                new Module(ModuleUtils.ModuleName.BACK_RIGHT),
        };
    }

    @Override
    protected String getLogPath() {
        return SwerveConstants.SWERVE_LOG_PATH;
    }

    @Override
    public void subsystemPeriodic() {
        ODOMETRY_LOCK.lock();
        updateAllInputs();
        ODOMETRY_LOCK.unlock();

        updatePoseEstimator();
        logState();
        logFieldRelativeVelocities();
    }

    private void updateAllInputs() {
        gyro.updateInputs(gyroInputs);
        Logger.processInputs(SwerveGyroConstants.LOG_PATH, gyroInputs);

        for (Module currentModule : modules) {
            currentModule.periodic();
        }
    }

    private void updatePoseEstimator() {
        POSE_ESTIMATOR.updatePoseEstimatorOdometry();
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


    public void stop() {
        for (Module currentModule : modules) {
            currentModule.stop();
        }
    }

    public void setBrake(boolean brake) {
        for (Module currentModule : modules) {
            currentModule.setBrake(brake);
        }
    }

    public void resetModulesAngleByEncoder() {
        for (Module module : getModules()) {
            module.resetByEncoder();
        }
    }

    protected void initializeDrive(SwerveState updatedState) {
        currentState.updateState(updatedState);
        setClosedLoopForModules();
        resetRotationController();
    }


    protected void setClosedLoopForModules() {
        for (Module currentModule : modules) {
            currentModule.setDriveMotorClosedLoop(currentState.getLoopMode().isClosedLoop);
        }
    }


    protected void resetRotationController() {
        SwerveConstants.ROTATION_PID_DEGREES_CONTROLLER.reset();
    }

    // Don't use this, only for pose estimation!!!
    public double[] getOdometryTimeStepQueue() {
        return gyroInputs.odometryUpdatesTimestamp;
    }

    // Don't use this, only for pose estimation!!!
    public Rotation2d[] getOdometryYawUpdates() {
        return gyroInputs.odometryUpdatesYaw;
    }

    public Rotation2d getAbsoluteHeading() {
        double inputtedHeadingRads = MathUtil.angleModulus(gyroInputs.gyroYaw.getRadians());
        return Rotation2d.fromRadians(inputtedHeadingRads);
    }

    public Rotation2d getRelativeHeading() {
        return Rotation2d.fromDegrees(gyroInputs.gyroYaw.getDegrees());
    }

    public void setHeading(Rotation2d heading) {
        gyro.setHeading(heading);
    }

    public Rotation2d getPitch() {
        return gyroInputs.gyroPitch;
    }

    public Translation3d getGyroAcceleration() {
        return new Translation3d(gyroInputs.accelerationX, gyroInputs.accelerationY, gyroInputs.accelerationZ);
    }


    public Rotation2d[] getModulesDriveDistances() {
        return Arrays.stream(modules).map(Module::getDriveDistanceAngle).toArray(Rotation2d[]::new);
    }

    public ChassisSpeeds getSelfRelativeVelocity() {
        return SwerveConstants.KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getFieldRelativeVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getSelfRelativeVelocity(), POSE_ESTIMATOR.getCurrentPose().getRotation());
    }


    /**
     * Runs swerve around itself for WheelRadiusCharacterization
     *
     * @param omegaPerSec - velocity to run the swerve
     */
    public void runWheelRadiusCharacterization(Rotation2d omegaPerSec) {
        driveByState(new ChassisSpeeds(0, 0, omegaPerSec.getRadians()));
    }

    /**
     * Runs swerve module around itself for Sysid Steer Calibration
     *
     * @param voltage - voltage to run the swerve module steer
     */
    public void runModuleSteerByVoltage(ModuleUtils.ModuleName module, double voltage) {
        modules[module.index].runSteerMotorByVoltage(voltage);
    }

    /**
     * Runs swerve module around itself for Sysid Steer Calibration
     *
     * @param voltage - voltage to run the swerve module drive
     */
    public void runModulesDriveByVoltage(double voltage) {
        for (Module module : modules) {
            module.runDriveMotorByVoltage(voltage);
        }
    }

    public void pointWheels(Rotation2d targetAngle, boolean optimize) {
        for (Module module : modules) {
            module.setTargetState(new SwerveModuleState(0, targetAngle), optimize);
        }
    }

    public void pointWheelsInX() {
        SwerveModuleState frontLeftBackRight = new SwerveModuleState(0, MathConstants.EIGHTH_CIRCLE);
        SwerveModuleState frontRightBackLeft = new SwerveModuleState(0, MathConstants.EIGHTH_CIRCLE.unaryMinus());

        modules[0].setTargetState(frontLeftBackRight);
        modules[1].setTargetState(frontRightBackLeft);
        modules[2].setTargetState(frontRightBackLeft);
        modules[3].setTargetState(frontLeftBackRight);
    }

    public void pointWheelsInCircle() {
        SwerveModuleState frontLeftBackRight = new SwerveModuleState(0, MathConstants.EIGHTH_CIRCLE.unaryMinus());
        SwerveModuleState frontRightBackLeft = new SwerveModuleState(0, MathConstants.EIGHTH_CIRCLE);

        modules[0].setTargetState(frontLeftBackRight);
        modules[1].setTargetState(frontRightBackLeft);
        modules[2].setTargetState(frontRightBackLeft);
        modules[3].setTargetState(frontLeftBackRight);
    }

    @AutoLogOutput(key = SwerveConstants.SWERVE_LOG_PATH + "IsModulesAtStates")
    public boolean isModulesAtStates() {
        for (Module module : modules) {
            if (!module.isAtTargetState()) {
                return false;
            }
        }
        return true;
    }

    @AutoLogOutput(key = SwerveConstants.SWERVE_LOG_PATH + "CurrentModulesStates")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getCurrentState();
        }

        return states;
    }

    @AutoLogOutput(key = SwerveConstants.SWERVE_LOG_PATH + "TargetModulesStates")
    private SwerveModuleState[] getTargetStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getTargetState();
        }

        return states;
    }

    public SwerveDriveWheelPositions getSwerveWheelPositions(int odometryUpdateIndex) {
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            swerveModulePositions[i] = modules[i].getOdometryPosition(odometryUpdateIndex);
        }
        return new SwerveDriveWheelPositions(swerveModulePositions);
    }

    private void setTargetModuleStates(SwerveModuleState[] swerveModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(swerveModuleStates[i]);
        }
    }


    protected void pidToPose(MirrorablePose2d targetPose) {
        Pose2d currentBluePose = POSE_ESTIMATOR.getCurrentPose();
        Pose2d mirroredTargetPose = targetPose.get();

        double xSpeed = SwerveConstants.TRANSLATION_PID_CONTROLLER.calculate(currentBluePose.getX(), mirroredTargetPose.getX());
        double ySpeed = SwerveConstants.TRANSLATION_PID_CONTROLLER.calculate(currentBluePose.getY(), mirroredTargetPose.getY());
        int direction = DriverStationUtils.isBlueAlliance() ? 1 : -1;
        Rotation2d thetaSpeed = calculateProfiledAngleSpeedToTargetAngle(targetPose.getRotation());

        ChassisSpeeds targetFieldRelativeSpeeds = new ChassisSpeeds(
                xSpeed * direction,
                ySpeed * direction,
                thetaSpeed.getRadians()
        );
        driveByState(targetFieldRelativeSpeeds);
    }

    protected void rotateToAngle(MirrorableRotation2d targetAngle) {
        ChassisSpeeds targetFieldRelativeSpeeds = new ChassisSpeeds(
                0,
                0,
                calculateProfiledAngleSpeedToTargetAngle(targetAngle).getRadians()
        );
        driveByState(targetFieldRelativeSpeeds);
    }

    // todo - maybe move some of work to SwerveMath class
    private Rotation2d calculateProfiledAngleSpeedToTargetAngle(MirrorableRotation2d targetAngle) {
        Rotation2d currentAngle = POSE_ESTIMATOR.getCurrentPose().getRotation();
        return Rotation2d.fromDegrees(SwerveConstants.ROTATION_PID_DEGREES_CONTROLLER.calculate(
                currentAngle.getDegrees(),
                targetAngle.get().getDegrees()
        ));
    }

    // todo - maybe move some of work to SwerveMath class
    private ChassisSpeeds applyAimAssistedRotationVelocity(ChassisSpeeds currentSpeeds, SwerveState swerveState) {
        if (swerveState.getAimAssist().equals(AimAssist.NONE)) {
            return currentSpeeds;
        }
        //PID
        Rotation2d pidVelocity = calculateProfiledAngleSpeedToTargetAngle(swerveState.getAimAssist().targetAngleSupplier.get());

        //Magnitude Factor
        double driveMagnitude = getDriveMagnitude(currentSpeeds);
        double angularVelocityRads =
                pidVelocity.getRadians() * SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR / (driveMagnitude + SwerveConstants.AIM_ASSIST_MAGNITUDE_FACTOR);

        //Joystick Output
        double angularVelocityWithJoystick = angularVelocityRads + currentSpeeds.omegaRadiansPerSecond;

        //Clamp
        double clampedAngularVelocity = MathUtil.clamp(
                angularVelocityWithJoystick,
                -SwerveConstants.MAX_ROTATIONAL_SPEED_PER_SECOND.getRadians(),
                SwerveConstants.MAX_ROTATIONAL_SPEED_PER_SECOND.getRadians()
        );

        //todo maybe - make value have stick range (P = MAX_ROT / MAX_ERROR = 10 rads / Math.PI) or clamp between MAX_ROT
        //todo - distance factor
        return new ChassisSpeeds(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond, clampedAngularVelocity);
    }

    private ChassisSpeeds getDriveModeRelativeChassisSpeeds(ChassisSpeeds chassisSpeeds, SwerveState swerveState) {
        if (swerveState.getDriveMode() == DriveMode.SELF_RELATIVE) {
            return chassisSpeeds;
        }
        else {
            return fieldRelativeSpeedsToSelfRelativeSpeeds(chassisSpeeds);
        }
    }

    //todo - move to drive mode or to SwerveMath class
    private ChassisSpeeds fieldRelativeSpeedsToSelfRelativeSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getDriveRelativeAngle());
    }

    // todo - maybe move some of work to SwerveMath class
    public Rotation2d getDriveRelativeAngle() {
        Rotation2d currentAngle = POSE_ESTIMATOR.getCurrentPose().getRotation();
        return DriverStationUtils.isRedAlliance() ? currentAngle.rotateBy(Rotation2d.fromDegrees(180)) : currentAngle;
    }

    // todo - maybe move some of work to SwerveMath class
    private ChassisSpeeds powersToSpeeds(double xPower, double yPower, double thetaPower) {
        return new ChassisSpeeds(
                xPower * currentState.getDriveSpeed().maxTranslationSpeedMetersPerSecond,
                yPower * currentState.getDriveSpeed().maxTranslationSpeedMetersPerSecond,
                thetaPower * currentState.getDriveSpeed().maxRotationSpeedPerSecond.getRadians()
        );
    }

    // todo: move to math
    private static double getDriveMagnitude(ChassisSpeeds chassisSpeeds){
        return Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond,2) + Math.pow(chassisSpeeds.vyMetersPerSecond,2));
    }

    /**
     * When the robot drives while rotating it skews a bit to the side.
     * This should fix the chassis speeds, so they won't make the robot skew while rotating.
     *
     * @param chassisSpeeds the chassis speeds to fix skewing for
     * @return the fixed speeds
     */
    // todo - maybe move some of work to SwerveMath class
    private ChassisSpeeds discretize(ChassisSpeeds chassisSpeeds) {
        return ChassisSpeeds.discretize(chassisSpeeds, CycleTimeUtils.getCurrentCycleTime());
    }


    protected void drive(double xPower, double yPower, double thetaPower) {
        driveByState(powersToSpeeds(xPower, yPower, thetaPower));
    }

    private void driveByState(ChassisSpeeds chassisSpeeds) {
        driveByState(chassisSpeeds, currentState);
    }

    public void driveByState(ChassisSpeeds chassisSpeeds, SwerveState swerveState) {
        chassisSpeeds = getDriveModeRelativeChassisSpeeds(chassisSpeeds, swerveState);

        chassisSpeeds = applyAimAssistedRotationVelocity(chassisSpeeds, swerveState);
        chassisSpeeds = discretize(chassisSpeeds);

        if (isStill(chassisSpeeds)) {
            stop();
            return;
        }

        SwerveModuleState[] swerveModuleStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(
                chassisSpeeds,
                swerveState.getRotateAxis().getRotateAxis()
        );
        setTargetModuleStates(swerveModuleStates);
    }

    // todo - maybe move some of work to SwerveMath class
    public boolean isStill(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.vxMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND
                && Math.abs(chassisSpeeds.vyMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND
                && Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= SwerveConstants.ROTATION_NEUTRAL_DEADBAND.getRadians();
    }

    // todo - maybe move some of work to SwerveMath class
    private boolean isAtTranslationPosition(double currentTranslationPosition, double targetTranslationPosition,
            double currentTranslationVelocity) {
        boolean isNearTargetPosition = MathUtil.isNear(
                targetTranslationPosition,
                currentTranslationPosition,
                SwerveConstants.TRANSLATION_TOLERANCE_METERS
        );
        boolean isStopping = Math.abs(currentTranslationVelocity) < SwerveConstants.TRANSLATION_VELOCITY_TOLERANCE;
        return isNearTargetPosition && isStopping;
    }

    // todo - maybe move some of work to SwerveMath class
    public boolean isAtXAxisPosition(double targetXBlueAlliancePosition) {
        double currentXAxisVelocity = getFieldRelativeVelocity().vxMetersPerSecond;
        return isAtTranslationPosition(
                POSE_ESTIMATOR.getCurrentPose().getX(),
                targetXBlueAlliancePosition,
                currentXAxisVelocity
        );
    }

    // todo - maybe move some of work to SwerveMath class
    public boolean isAtYAxisPosition(double targetYBlueAlliancePosition) {
        double currentYAxisVelocity = getFieldRelativeVelocity().vyMetersPerSecond;
        return isAtTranslationPosition(
                POSE_ESTIMATOR.getCurrentPose().getY(),
                targetYBlueAlliancePosition,
                currentYAxisVelocity
        );
    }

    // todo - maybe move some of work to SwerveMath class
    public boolean isAtAngle(MirrorableRotation2d targetAngle) {
        double angleDifferenceDeg = Math.abs(targetAngle.get().minus(POSE_ESTIMATOR.getCurrentPose().getRotation()).getDegrees());
        boolean isAtAngle = angleDifferenceDeg < SwerveConstants.ROTATION_TOLERANCE.getDegrees();

        double currentRotationVelocityRadians = getSelfRelativeVelocity().omegaRadiansPerSecond;
        boolean isStopping = Math.abs(currentRotationVelocityRadians) < SwerveConstants.ROTATION_VELOCITY_TOLERANCE.getRadians();

        return isAtAngle && isStopping;
    }

    // todo - maybe move some of work to SwerveMath class
    public boolean isAtPosition(MirrorablePose2d targetPose) {
        Pose2d targetBluePose = targetPose.get();
        return isAtXAxisPosition(targetBluePose.getX())
                && isAtYAxisPosition(targetBluePose.getY())
                && isAtAngle(targetPose.getRotation()
        );
    }

}
