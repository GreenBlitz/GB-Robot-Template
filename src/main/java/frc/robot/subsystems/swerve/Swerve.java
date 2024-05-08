package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.MathConstants;
import frc.robot.subsystems.swerve.modules.Module;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.swervegyro.swervegyrointerface.ISwerveGyro;
import frc.robot.subsystems.swerve.swervegyro.swervegyrointerface.SwerveGyroFactory;
import frc.robot.subsystems.swerve.swervegyro.swervegyrointerface.SwerveGyroInputsAutoLogged;
import frc.utils.DriverStationUtils;
import frc.utils.GBSubsystem;
import frc.utils.allianceutils.AlliancePose2d;
import frc.utils.allianceutils.AllianceUtils;
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

    private DriveMode driveMode;


    public Swerve() {
        setName("Swerve");
        driveMode = DriveMode.NORMAL;

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
                () -> POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose(),
                (pose) -> POSE_ESTIMATOR.resetPose(AlliancePose2d.fromBlueAlliancePose(pose)),
                this::getSelfRelativeVelocity,
                this::selfRelativeDrive,
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

    //todo - make it get "SwerveState"
    protected void initializeDrive(boolean closedLoop, DriveMode wantedDriveMode) {
        setDriveMode(wantedDriveMode);
        initializeDrive(closedLoop);
    }

    protected void initializeDrive(boolean closedLoop) {
        setClosedLoop(closedLoop);
        resetRotationController();
    }

    protected void setClosedLoop(boolean closedLoop) {
        for (Module currentModule : modules) {
            currentModule.setDriveMotorClosedLoop(closedLoop);
        }
    }

    protected void setDriveMode(DriveMode wantedDriveMode) {
        driveMode = wantedDriveMode;
    }

    protected void resetRotationController() {
        SwerveConstants.PROFILED_ROTATION_PID_DEGREES_CONTROLLER.reset(
                POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose().getRotation().getDegrees()
        );
    }


    public double[] getOdometryTimeStepQueue() {
        return gyroInputs.odometryUpdatesTimestamp;
    }

    public Rotation2d[] getOdometryYawUpdates() {
        return gyroInputs.odometryUpdatesYaw;
    }

    public Rotation2d getAbsoluteHeading() {
        double inputtedHeading = MathUtil.inputModulus(
                gyroInputs.gyroYaw.getDegrees(),
                -MathConstants.HALF_CIRCLE.getDegrees(),
                MathConstants.HALF_CIRCLE.getDegrees()
        );
        return Rotation2d.fromDegrees(inputtedHeading);
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
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                getSelfRelativeVelocity(),
                POSE_ESTIMATOR.getCurrentPose().toAlliancePose().getRotation()
        );
    }


    /**
     * Runs swerve around itself for WheelRadiusCharacterization
     *
     * @param omegaPerSec - velocity to run the swerve
     */
    public void runWheelRadiusCharacterization(Rotation2d omegaPerSec) {
        selfRelativeDrive(new ChassisSpeeds(0, 0, omegaPerSec.getRadians()));
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


    protected void pidToPose(Pose2d targetPose) {
        Pose2d currentPose = POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose();
        double xSpeed = SwerveConstants.TRANSLATION_PID_CONTROLLER.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = SwerveConstants.TRANSLATION_PID_CONTROLLER.calculate(currentPose.getY(), targetPose.getY());
        int direction = DriverStationUtils.isBlueAlliance() ? 1 : -1;
        ChassisSpeeds targetFieldRelativeSpeeds = new ChassisSpeeds(
                xSpeed * direction,
                ySpeed * direction,
                calculateProfiledAngleSpeedToTargetAngle(targetPose.getRotation())
        );
        selfRelativeDrive(fieldRelativeSpeedsToSelfRelativeSpeeds(targetFieldRelativeSpeeds));
    }

    protected void rotateToAngle(Rotation2d targetAngle) {
        ChassisSpeeds targetFieldRelativeSpeeds = new ChassisSpeeds(
                0,
                0,
                calculateProfiledAngleSpeedToTargetAngle(targetAngle)
        );
        selfRelativeDrive(fieldRelativeSpeedsToSelfRelativeSpeeds(targetFieldRelativeSpeeds));
    }

    //todo - make it use "SwerveRotationAxis"
    protected void rotateToAngleAroundWheel(Rotation2d targetAngle, ModuleUtils.ModuleName moduleName) {
        ChassisSpeeds targetFieldRelativeSpeeds = new ChassisSpeeds(
                0,
                0,
                calculateProfiledAngleSpeedToTargetAngle(targetAngle)
                // todo - check if needs another pid controller
                //  need to be checked on carpet
        );
        selfRelativeDriveAndRotateAroundWantedPoint(fieldRelativeSpeedsToSelfRelativeSpeeds(targetFieldRelativeSpeeds), moduleName);
    }

    private double calculateProfiledAngleSpeedToTargetAngle(Rotation2d targetAngle) {
        Rotation2d currentAngle = POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose().getRotation();
        return Units.degreesToRadians(SwerveConstants.PROFILED_ROTATION_PID_DEGREES_CONTROLLER.calculate(
                currentAngle.getDegrees(),
                targetAngle.getDegrees()
        ));
    }


    private ChassisSpeeds fieldRelativeSpeedsToSelfRelativeSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
        Rotation2d currentAngle = POSE_ESTIMATOR.getCurrentPose().toAlliancePose().getRotation();
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, currentAngle);
    }

    private ChassisSpeeds selfRelativeSpeedsFromFieldRelativePowers(double xPower, double yPower, double thetaPower) {
        ChassisSpeeds fieldRelativeSpeeds = powersToSpeeds(xPower, yPower, thetaPower);
        return fieldRelativeSpeedsToSelfRelativeSpeeds(fieldRelativeSpeeds);
    }

    private ChassisSpeeds powersToSpeeds(double xPower, double yPower, double thetaPower) {
        return new ChassisSpeeds(
                xPower * driveMode.maxTranslationSpeedMetersPerSecond,
                yPower * driveMode.maxTranslationSpeedMetersPerSecond,
                Math.pow(thetaPower, 2) * Math.signum(thetaPower) * driveMode.maxRotationSpeedPerSecond.getRadians()
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


    /**
     * Drives the swerve with the given powers, relative to the field's frame of reference, and rotating around given module
     * instead of middle of robot.
     *
     * @param xPower the x power
     * @param yPower the y power
     * @param thetaPower the theta power
     * @param moduleToTurnAround the module to turn around
     */
    //todo - make it use "SwerveRotationAxis"
    protected void fieldRelativeDriveRotateAroundModule(
            double xPower, double yPower, double thetaPower, ModuleUtils.ModuleName moduleToTurnAround
    ) {
        ChassisSpeeds speeds = selfRelativeSpeedsFromFieldRelativePowers(xPower, yPower, thetaPower);
        selfRelativeDriveAndRotateAroundWantedPoint(speeds, moduleToTurnAround);
    }

    /**
     * Drives the swerve with the given powers and a target angle, relative to the field's frame of reference.
     *
     * @param xPower the x power
     * @param yPower the y power
     * @param targetAngle the target angle, relative to the blue alliance's forward position
     */
    protected void fieldRelativeDrive(double xPower, double yPower, Rotation2d targetAngle) {
        targetAngle = AllianceUtils.toMirroredAllianceRotation(targetAngle);
        ChassisSpeeds speeds = selfRelativeSpeedsFromFieldRelativePowers(xPower, yPower, 0);
        speeds.omegaRadiansPerSecond = calculateProfiledAngleSpeedToTargetAngle(targetAngle);

        selfRelativeDrive(speeds);
    }

    /**
     * Drives the swerve with the given powers, relative to the field's frame of reference.
     *
     * @param xPower the x power
     * @param yPower the y power
     * @param thetaPower the theta power
     */
    protected void fieldRelativeDrive(double xPower, double yPower, double thetaPower) {
        ChassisSpeeds speeds = selfRelativeSpeedsFromFieldRelativePowers(xPower, yPower, thetaPower);
        selfRelativeDrive(speeds);
    }


    /**
     * Drives the swerve with the given powers, relative to the robot's frame of reference, and rotating around given module
     * instead of middle of robot.
     *
     * @param xPower the x power
     * @param yPower the y power
     * @param thetaPower the theta power
     * @param moduleToTurnAround the module to turn around
     */
    protected void selfRelativeDriveRotateAroundModule(
            double xPower, double yPower, double thetaPower, ModuleUtils.ModuleName moduleToTurnAround
    ) {
        ChassisSpeeds speeds = powersToSpeeds(xPower, yPower, thetaPower);
        selfRelativeDriveAndRotateAroundWantedPoint(speeds, moduleToTurnAround);
    }

    /**
     * Drives the swerve with the given powers and a target angle, relative to the robot's frame of reference.
     *
     * @param xPower the x power
     * @param yPower the y power
     * @param targetAngle the target angle
     */
    protected void selfRelativeDrive(double xPower, double yPower, Rotation2d targetAngle) {
        ChassisSpeeds speeds = powersToSpeeds(xPower, yPower, 0);
        speeds.omegaRadiansPerSecond = calculateProfiledAngleSpeedToTargetAngle(targetAngle);

        selfRelativeDrive(speeds);
    }

    /**
     * Drives the swerve with the given powers, relative to the robot's frame of reference.
     *
     * @param xPower the x power
     * @param yPower the y power
     * @param thetaPower the theta power
     */
    protected void selfRelativeDrive(double xPower, double yPower, double thetaPower) {
        ChassisSpeeds speeds = powersToSpeeds(xPower, yPower, thetaPower);
        selfRelativeDrive(speeds);
    }

    /**
     * Drives the swerve with the given speeds, rotating around given Module instead of middle of robot.
     *
     * @param chassisSpeeds the speeds to drive at
     * @param moduleToTurnAround the Module to turn around
     */
    private void selfRelativeDriveAndRotateAroundWantedPoint(
            ChassisSpeeds chassisSpeeds, ModuleUtils.ModuleName moduleToTurnAround
    ) {
        selfRelativeDriveAndRotateAroundWantedPoint(
                chassisSpeeds,
                ModuleUtils.getModulePositionRelativeToMiddleOfRobot(moduleToTurnAround)
        );
    }

    /**
     * Drives the swerve with the given speeds, rotating around given Translation2D instead of middle of robot.
     *
     * @param chassisSpeeds the speeds to drive at
     * @param positionToTurnAround the Translation2D to turn around
     */
    private void selfRelativeDriveAndRotateAroundWantedPoint(
            ChassisSpeeds chassisSpeeds, Translation2d positionToTurnAround
    ) {
        chassisSpeeds = discretize(chassisSpeeds);
        if (isStill(chassisSpeeds)) {
            stop();
            return;
        }

        SwerveModuleState[] swerveModuleStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds, positionToTurnAround);
        setTargetModuleStates(swerveModuleStates);
    }

    private void selfRelativeDrive(ChassisSpeeds chassisSpeeds) {
        chassisSpeeds = discretize(chassisSpeeds);
        if (isStill(chassisSpeeds)) {
            stop();
            return;
        }

        SwerveModuleState[] swerveModuleStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        setTargetModuleStates(swerveModuleStates);
    }


    public boolean isStill(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.vxMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND && Math.abs(chassisSpeeds.vyMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND && Math.abs(
                chassisSpeeds.omegaRadiansPerSecond) <= SwerveConstants.ROTATION_NEUTRAL_DEADBAND;
    }

    private boolean isAtTranslationPosition(
            double currentTranslationPosition, double targetTranslationPosition, double currentTranslationVelocity
    ) {
        return Math.abs(currentTranslationPosition - targetTranslationPosition) < SwerveConstants.TRANSLATION_TOLERANCE_METERS && Math.abs(
                currentTranslationVelocity) < SwerveConstants.TRANSLATION_VELOCITY_TOLERANCE;
    }

    public boolean isAtXAxisPosition(double targetXAxisPosition) {
        double currentXAxisVelocity = getFieldRelativeVelocity().vxMetersPerSecond;
        return isAtTranslationPosition(
                POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose().getX(),
                targetXAxisPosition,
                currentXAxisVelocity
        );
    }

    public boolean isAtYAxisPosition(double targetYAxisPosition) {
        double currentYAxisVelocity = getFieldRelativeVelocity().vyMetersPerSecond;
        return isAtTranslationPosition(
                POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose().getY(),
                targetYAxisPosition,
                currentYAxisVelocity
        );
    }

    public boolean isAtAngle(Rotation2d targetAngle) {
        double angleDifferenceDeg = Math.abs(
                targetAngle.minus(POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose().getRotation()).getDegrees()
        );
        boolean isAtAngle = angleDifferenceDeg < SwerveConstants.ROTATION_TOLERANCE.getDegrees();

        double currentRotationVelocityRadians = getSelfRelativeVelocity().omegaRadiansPerSecond;
        boolean isStopping = Math.abs(currentRotationVelocityRadians) < SwerveConstants.ROTATION_VELOCITY_TOLERANCE.getRadians();

        return isAtAngle && isStopping;
    }

    public boolean isAtPosition(Pose2d targetPose) {
        return isAtXAxisPosition(targetPose.getX()) && isAtYAxisPosition(targetPose.getY()) && isAtAngle(targetPose.getRotation());
    }

}
