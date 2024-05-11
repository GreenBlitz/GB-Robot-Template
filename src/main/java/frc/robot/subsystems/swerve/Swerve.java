package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
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
import frc.robot.subsystems.swerve.swervegyro.swervegyrointerface.ISwerveGyro;
import frc.robot.subsystems.swerve.swervegyro.swervegyrointerface.SwerveGyroFactory;
import frc.robot.subsystems.swerve.swervegyro.swervegyrointerface.SwerveGyroInputsAutoLogged;
import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import frc.utils.DriverStationUtils;
import frc.utils.GBSubsystem;
import frc.utils.allianceutils.AlliancePose2d;
import frc.utils.allianceutils.AllianceRotation2d;
import frc.utils.roborioutils.RoborioUtils;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;

// todo - add aim assist calculations (maybe in the "SwerveAimAssist" class)
public class Swerve extends GBSubsystem {

    public static final Lock ODOMETRY_LOCK = new ReentrantLock();

    private final SwerveGyroInputsAutoLogged gyroInputs;

    private final ISwerveGyro gyro;

    private final Module[] modules;
    private final SwerveState currentState;


    public Swerve() {
        setName("Swerve");
        currentState = new SwerveState();

        gyro = SwerveGyroFactory.createSwerve();
        modules = getModules();

        gyroInputs = new SwerveGyroInputsAutoLogged();

        configurePathPlanner();
    }

    private Module[] getModules() {
        return new Module[]{
                new Module(ModuleUtils.ModuleName.FRONT_LEFT),
                new Module(ModuleUtils.ModuleName.FRONT_RIGHT),
                new Module(ModuleUtils.ModuleName.BACK_LEFT),
                new Module(ModuleUtils.ModuleName.BACK_RIGHT),
        };
    }

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
                () -> POSE_ESTIMATOR.getCurrentPose().getBlueAlliancePose(),
                (pose) -> POSE_ESTIMATOR.resetPose(AlliancePose2d.fromBlueAlliancePose(pose)),
                this::getSelfRelativeVelocity,
                this::driveByState,
                SwerveConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG,
                () -> !DriverStationUtils.isBlueAlliance(),
                this
        );
    }


    @Override
    public void periodic() {
        ODOMETRY_LOCK.lock();
        updateAllInputs();
        ODOMETRY_LOCK.unlock();

        updatePoseEstimator();
        updateNetworkTables();
    }

    private void updateAllInputs() {
        gyro.updateInputs(gyroInputs);
        Logger.processInputs("Gyro", gyroInputs);

        for (Module currentModule : modules) {
            currentModule.periodic();
        }
    }

    private void updatePoseEstimator() {
        POSE_ESTIMATOR.updatePoseEstimatorOdometry();
    }


    private void updateNetworkTables() {
        Logger.recordOutput("Swerve/Velocity/Rotation", getSelfRelativeVelocity().omegaRadiansPerSecond);
        Logger.recordOutput("Swerve/Velocity/X", getSelfRelativeVelocity().vxMetersPerSecond);
        Logger.recordOutput("Swerve/Velocity/Y", getSelfRelativeVelocity().vyMetersPerSecond);
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
        SwerveConstants.PROFILED_ROTATION_PID_DEGREES_CONTROLLER.reset(
                POSE_ESTIMATOR.getCurrentPose().getBlueAlliancePose().getRotation().getDegrees()
        );
    }

    // Don't use this, only for pose estimation!!!
    public double[] getOdometryTimeStepQueue() {
        return gyroInputs.odometryUpdatesTimestamp;
    }

    // Don't use this, only for pose estimation!!!
    public Rotation2d[] getOdometryYawUpdates() {
        return gyroInputs.odometryUpdatesYaw;
    }

    public AllianceRotation2d getAbsoluteHeading() {
        double inputtedHeading = MathUtil.inputModulus(
                gyroInputs.gyroYaw.getDegrees(),
                -MathConstants.HALF_CIRCLE.getDegrees(),
                MathConstants.HALF_CIRCLE.getDegrees()
        );
        return AllianceRotation2d.fromBlueAllianceRotation(Rotation2d.fromDegrees(inputtedHeading));
    }

    public AllianceRotation2d getRelativeHeading() {
        return AllianceRotation2d.fromBlueAllianceRotation(Rotation2d.fromDegrees(gyroInputs.gyroYaw.getDegrees()));
    }

    public void setHeading(AllianceRotation2d heading) {
        gyro.setHeading(heading.getBlueAllianceAngle());
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
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                getSelfRelativeVelocity(),
                POSE_ESTIMATOR.getCurrentPose().getAlliancePose().getRotation()
        );
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
     * Point all wheels in same angle
     *
     * @param targetAngle - angle to point to
     */
    public void pointWheels(Rotation2d targetAngle) {
        for (Module module : modules) {
            module.setTargetState(new SwerveModuleState(0, targetAngle));
        }
    }

    /**
     * Lock swerve wheels in X position, so it's hard to move it.
     */
    public void lockSwerve() {
        SwerveModuleState frontLeftBackRight = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
        SwerveModuleState frontRightBackLeft = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));

        modules[0].setTargetState(frontLeftBackRight);
        modules[1].setTargetState(frontRightBackLeft);
        modules[2].setTargetState(frontRightBackLeft);
        modules[3].setTargetState(frontLeftBackRight);
    }

    public boolean isModulesAtStates() {
        boolean isAtStates = true;
        for (Module module : modules) {
            isAtStates = isAtStates && module.isAtTargetState();
        }
        return isAtStates;
    }

    @AutoLogOutput(key = "Swerve/CurrentStates")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getCurrentState();
        }

        return states;
    }

    @AutoLogOutput(key = "Swerve/TargetStates")
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


    protected void pidToPose(AlliancePose2d targetPose) {
        Pose2d currentBluePose = POSE_ESTIMATOR.getCurrentPose().getBlueAlliancePose();
        double xSpeed = SwerveConstants.TRANSLATION_PID_CONTROLLER.calculate(
                currentBluePose.getX(),
                targetPose.getBlueAlliancePose().getX()
        );
        double ySpeed = SwerveConstants.TRANSLATION_PID_CONTROLLER.calculate(
                currentBluePose.getY(),
                targetPose.getBlueAlliancePose().getY()
        );
        int direction = DriverStationUtils.isBlueAlliance() ? 1 : -1;
        ChassisSpeeds targetFieldRelativeSpeeds = new ChassisSpeeds(
                xSpeed * direction,
                ySpeed * direction,
                calculateProfiledAngleSpeedToTargetAngle(targetPose.getRotation2d()).getRadians()
        );
        driveByState(targetFieldRelativeSpeeds);
    }

    protected void rotateToAngle(AllianceRotation2d targetAngle) {
        ChassisSpeeds targetFieldRelativeSpeeds = new ChassisSpeeds(
                0,
                0,
                calculateProfiledAngleSpeedToTargetAngle(targetAngle).getRadians()
        );
        driveByState(targetFieldRelativeSpeeds);
    }

    private Rotation2d calculateProfiledAngleSpeedToTargetAngle(AllianceRotation2d targetAngle) {
        AllianceRotation2d currentAngle = POSE_ESTIMATOR.getCurrentPose().getRotation2d();
        return Rotation2d.fromDegrees(SwerveConstants.PROFILED_ROTATION_PID_DEGREES_CONTROLLER.calculate(
                currentAngle.getBlueAllianceAngle().getDegrees(),
                targetAngle.getBlueAllianceAngle().getDegrees()
        ));
    }


    private Rotation2d getAimAssistThetaVelocity() {
        if (currentState.getAimAssist().equals(AimAssist.NONE)) {
            return new Rotation2d();
        }
        Rotation2d pidVelocity = calculateProfiledAngleSpeedToTargetAngle(currentState.getAimAssist().targetAngleSupplier.get());
        //todo - make value have same range like joystick
        //todo - distance factor
        //todo - current robot velocity factor
        return pidVelocity;
    }

    //todo - move to drive mode
    public static ChassisSpeeds fieldRelativeSpeedsToSelfRelativeSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
        Rotation2d currentAngle = POSE_ESTIMATOR.getCurrentPose().getAlliancePose().getRotation();
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, currentAngle);
    }

    private ChassisSpeeds powersToSpeeds(double xPower, double yPower, double thetaPower) {
        return new ChassisSpeeds(
                xPower * currentState.getDriveSpeed().maxTranslationSpeedMetersPerSecond,
                yPower * currentState.getDriveSpeed().maxTranslationSpeedMetersPerSecond,
                Math.pow(thetaPower, 2) * Math.signum(thetaPower) * currentState.getDriveSpeed().maxRotationSpeedPerSecond.getRadians()
        );
    }

    /**
     * When the robot drives while rotating it skews a bit to the side.
     * This should fix the chassis speeds, so they won't make the robot skew while rotating.
     *
     * @param chassisSpeeds the chassis speeds to fix skewing for
     * @return the fixed speeds
     */
    private ChassisSpeeds discretize(ChassisSpeeds chassisSpeeds) {
        return ChassisSpeeds.discretize(chassisSpeeds, RoborioUtils.getCurrentRoborioCycleTime());
    }

    protected void drive(double xPower, double yPower, double thetaPower) {
        driveByState(powersToSpeeds(xPower, yPower, thetaPower));
    }

    private void driveByState(ChassisSpeeds chassisSpeeds) {
        chassisSpeeds = currentState.getDriveMode().getDriveModeRelativeChassisSpeeds(chassisSpeeds);

        chassisSpeeds.omegaRadiansPerSecond += getAimAssistThetaVelocity().getRadians();//todo - clamp
        chassisSpeeds = discretize(chassisSpeeds);

        if (isStill(chassisSpeeds)) {
            stop();
            return;
        }

        SwerveModuleState[] swerveModuleStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(
                chassisSpeeds,
                currentState.getRotateAxis().getRotateAxis()
        );
        setTargetModuleStates(swerveModuleStates);
    }


    public boolean isStill(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.vxMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND
                && Math.abs(chassisSpeeds.vyMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND
                && Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= SwerveConstants.ROTATION_NEUTRAL_DEADBAND;
    }

    private boolean isAtTranslationPosition(
            double currentTranslationPosition, double targetTranslationPosition, double currentTranslationVelocity
    ) {//todo - clean
        return Math.abs(currentTranslationPosition - targetTranslationPosition) < SwerveConstants.TRANSLATION_TOLERANCE_METERS && Math.abs(
                currentTranslationVelocity) < SwerveConstants.TRANSLATION_VELOCITY_TOLERANCE;
    }

    public boolean isAtXAxisPosition(double targetXBlueAlliancePosition) {
        double currentXAxisVelocity = getFieldRelativeVelocity().vxMetersPerSecond;
        return isAtTranslationPosition(
                POSE_ESTIMATOR.getCurrentPose().getBlueAlliancePose().getX(),
                targetXBlueAlliancePosition,
                currentXAxisVelocity
        );
    }

    public boolean isAtYAxisPosition(double targetYBlueAlliancePosition) {
        double currentYAxisVelocity = getFieldRelativeVelocity().vyMetersPerSecond;
        return isAtTranslationPosition(
                POSE_ESTIMATOR.getCurrentPose().getBlueAlliancePose().getY(),
                targetYBlueAlliancePosition,
                currentYAxisVelocity
        );
    }

    public boolean isAtAngle(AllianceRotation2d targetAngle) {
        double angleDifferenceDeg = Math.abs(
                targetAngle.getBlueAllianceAngle()
                           .minus(POSE_ESTIMATOR.getCurrentPose().getBlueAlliancePose().getRotation())
                           .getDegrees()
        );
        boolean isAtAngle = angleDifferenceDeg < SwerveConstants.ROTATION_TOLERANCE.getDegrees();

        double currentRotationVelocityRadians = getSelfRelativeVelocity().omegaRadiansPerSecond;
        boolean isStopping = Math.abs(currentRotationVelocityRadians) < SwerveConstants.ROTATION_VELOCITY_TOLERANCE.getRadians();

        return isAtAngle && isStopping;
    }

    public boolean isAtPosition(AlliancePose2d targetPose) {
        return isAtXAxisPosition(targetPose.getBlueAlliancePose().getX())
                && isAtYAxisPosition(targetPose.getBlueAlliancePose().getY())
                && isAtAngle(targetPose.getRotation2d()
        );
    }

}
