package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import frc.utils.DriverStationUtils;
import frc.utils.GBSubsystem;
import frc.utils.RobotTypeUtils;
import frc.utils.allianceutils.AlliancePose2d;
import frc.utils.allianceutils.AllianceUtils;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Swerve extends GBSubsystem {
    public final Lock odometryLock;
    private final SwerveInputsAutoLogged swerveInputs;
    private final SwerveIO swerveIO;
    private final SwerveConstants constants;
    private final SwerveModuleIO[] modulesIO;

    public Swerve() {
        setName("Swerve");

        swerveIO = SwerveIO.generateIO();
        modulesIO = getModulesIO();
        constants = SwerveConstants.generateConstants();

        swerveInputs = new SwerveInputsAutoLogged();
        odometryLock = new ReentrantLock();

        constants.getProfiledRotationController().enableContinuousInput(-180, 180);
        configurePathPlanner();
    }

    private SwerveModuleIO[] getModulesIO() {
        if (RobotTypeUtils.isReplay()) {
            return new SwerveModuleIO[]{
                    new SwerveModuleIO("FrontLeft"),
                    new SwerveModuleIO("FrontRight"),
                    new SwerveModuleIO("RearLeft"),
                    new SwerveModuleIO("RearRight")
            };
        }
        return constants.getModulesIO().get();
    }

    private void configurePathPlanner() {
        AutoBuilder.configureHolonomic(
                () -> RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose(),
                (pose) -> RobotContainer.POSE_ESTIMATOR.resetPose(AlliancePose2d.fromBlueAlliancePose(pose)),
                this::getSelfRelativeVelocity,
                this::selfRelativeDrive,
                constants.getPathFollowerConfig(),
                () -> !DriverStationUtils.isBlueAlliance(),
                this
        );
    }

    public SwerveConstants getConstants() {
        return constants;
    }

    @Override
    public void periodic() {
        odometryLock.lock();
        swerveIO.updateInputs(swerveInputs);
        Logger.processInputs(this.getName(), swerveInputs);

        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.periodic();
        odometryLock.unlock();

        updatePoseEstimatorStates();
        updateNetworkTables();
    }


    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(swerveInputs.gyroPitchDegrees);
    }

    public Rotation2d getHeading() {
        final double inputtedHeading = MathUtil.inputModulus(swerveInputs.gyroYawDegrees, -180, 180);
        return Rotation2d.fromDegrees(inputtedHeading);
    }

    public void setHeading(Rotation2d heading) {
        swerveIO.setHeading(heading);
    }

    public Translation3d getGyroAcceleration() {
        return new Translation3d(swerveInputs.accelerationX, swerveInputs.accelerationY, swerveInputs.accelerationZ);
    }


    public void stop() {
        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.stop();
    }

    public void setBrake(boolean brake) {
        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.setBrake(brake);
    }


    public ChassisSpeeds getSelfRelativeVelocity() {
        return constants.getKinematics().toChassisSpeeds(getModuleStates());
    }

    public ChassisSpeeds getFieldRelativeVelocity() {
        return ChassisSpeeds.fromFieldRelativeSpeeds(getSelfRelativeVelocity(), RobotContainer.POSE_ESTIMATOR.getCurrentPose().toAlliancePose().getRotation());
    }


    void pidToPose(Pose2d targetPose) {
        final Pose2d currentPose = RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose();
        final ChassisSpeeds targetFieldRelativeSpeeds = new ChassisSpeeds(
                constants.getTranslationsController().calculate(currentPose.getX(), targetPose.getX()),
                constants.getTranslationsController().calculate(currentPose.getY(), targetPose.getY()),
                calculateProfiledAngleSpeedToTargetAngle(targetPose.getRotation())
        );
        selfRelativeDrive(fieldRelativeSpeedsToSelfRelativeSpeeds(targetFieldRelativeSpeeds));
    }

    void initializeDrive(boolean closedLoop) {
        setClosedLoop(closedLoop);
        resetRotationController();
    }

    void resetRotationController() {
        constants.getProfiledRotationController().reset(RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose().getRotation().getDegrees());
    }

    void setClosedLoop(boolean closedLoop) {
        for (SwerveModuleIO currentModule : modulesIO)
            currentModule.setDriveMotorClosedLoop(closedLoop);
    }

    /**
     * Drives the swerve with the given powers and a target angle, relative to the field's frame of reference.
     *
     * @param xPower      the x power
     * @param yPower      the y power
     * @param targetAngle the target angle, relative to the blue alliance's forward position
     */
    void fieldRelativeDrive(double xPower, double yPower, Rotation2d targetAngle) {
        targetAngle = AllianceUtils.toMirroredAllianceRotation(targetAngle);
        final ChassisSpeeds speeds = selfRelativeSpeedsFromFieldRelativePowers(xPower, yPower, 0);
        speeds.omegaRadiansPerSecond = calculateProfiledAngleSpeedToTargetAngle(targetAngle);

        selfRelativeDrive(speeds);
    }

    /**
     * Drives the swerve with the given powers, relative to the field's frame of reference.
     *
     * @param xPower     the x power
     * @param yPower     the y power
     * @param thetaPower the theta power
     */
    void fieldRelativeDrive(double xPower, double yPower, double thetaPower) {
        final ChassisSpeeds speeds = selfRelativeSpeedsFromFieldRelativePowers(xPower, yPower, thetaPower);
        selfRelativeDrive(speeds);
    }

    /**
     * Drives the swerve with the given powers, relative to the robot's frame of reference.
     *
     * @param xPower     the x power
     * @param yPower     the y power
     * @param thetaPower the theta power
     */
    void selfRelativeDrive(double xPower, double yPower, double thetaPower) {
        final ChassisSpeeds speeds = powersToSpeeds(xPower, yPower, thetaPower);
        selfRelativeDrive(speeds);
    }

    /**
     * Drives the swerve with the given powers and a target angle, relative to the robot's frame of reference.
     *
     * @param xPower      the x power
     * @param yPower      the y power
     * @param targetAngle the target angle
     */
    void selfRelativeDrive(double xPower, double yPower, Rotation2d targetAngle) {
        final ChassisSpeeds speeds = powersToSpeeds(xPower, yPower, 0);
        speeds.omegaRadiansPerSecond = calculateProfiledAngleSpeedToTargetAngle(targetAngle);

        selfRelativeDrive(speeds);
    }

    private void selfRelativeDrive(ChassisSpeeds chassisSpeeds) {
        chassisSpeeds = discretize(chassisSpeeds);
        if (isStill(chassisSpeeds)) {
            stop();
            return;
        }

        final SwerveModuleState[] swerveModuleStates = constants.getKinematics().toSwerveModuleStates(chassisSpeeds);
        setTargetModuleStates(swerveModuleStates);
    }

    private void setTargetModuleStates(SwerveModuleState[] swerveModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, constants.getMaxSpeedMetersPerSecond());
        for (int i = 0; i < modulesIO.length; i++)
            modulesIO[i].setTargetState(swerveModuleStates[i]);
    }

    /**
     * When the robot drives while rotating it skews a bit to the side.
     * This should fix the chassis speeds, so they won't make the robot skew while rotating.
     *
     * @param chassisSpeeds the chassis speeds to fix skewing for
     * @return the fixed speeds
     */
    private ChassisSpeeds discretize(ChassisSpeeds chassisSpeeds) {
        return ChassisSpeeds.discretize(chassisSpeeds, RobotConstants.PERIODIC_TIME_SECONDS);
    }

    private void updatePoseEstimatorStates() {
        final int odometryUpdates = swerveInputs.odometryUpdatesYawDegrees.length;
        final SwerveDriveWheelPositions[] swerveWheelPositions = new SwerveDriveWheelPositions[odometryUpdates];
        final Rotation2d[] gyroRotations = new Rotation2d[odometryUpdates];

        for (int i = 0; i < odometryUpdates; i++) {
            swerveWheelPositions[i] = getSwerveWheelPositions(i);
            gyroRotations[i] = Rotation2d.fromDegrees(swerveInputs.odometryUpdatesYawDegrees[i]);

        }

        RobotContainer.POSE_ESTIMATOR.updatePoseEstimatorStates(swerveWheelPositions, gyroRotations, swerveInputs.odometryUpdatesTimestamp);
    }

    private SwerveDriveWheelPositions getSwerveWheelPositions(int odometryUpdateIndex) {
        final SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[modulesIO.length];
        for (int i = 0; i < modulesIO.length; i++)
            swerveModulePositions[i] = modulesIO[i].getOdometryPosition(odometryUpdateIndex);
        return new SwerveDriveWheelPositions(swerveModulePositions);
    }



    private double calculateProfiledAngleSpeedToTargetAngle(Rotation2d targetAngle) {
        final Rotation2d currentAngle = RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose().getRotation();
        return Units.degreesToRadians(constants.getProfiledRotationController().calculate(currentAngle.getDegrees(), targetAngle.getDegrees()));
    }

    private ChassisSpeeds selfRelativeSpeedsFromFieldRelativePowers(double xPower, double yPower, double thetaPower) {
        final ChassisSpeeds fieldRelativeSpeeds = powersToSpeeds(xPower, yPower, thetaPower);
        return fieldRelativeSpeedsToSelfRelativeSpeeds(fieldRelativeSpeeds);
    }

    private ChassisSpeeds fieldRelativeSpeedsToSelfRelativeSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
        final Rotation2d currentAngle = RobotContainer.POSE_ESTIMATOR.getCurrentPose().toAlliancePose().getRotation();
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, currentAngle);
    }

    private ChassisSpeeds powersToSpeeds(double xPower, double yPower, double thetaPower) {
        return new ChassisSpeeds(
                xPower * constants.getMaxSpeedMetersPerSecond(),
                yPower * constants.getMaxSpeedMetersPerSecond(),
                Math.pow(thetaPower, 2) * Math.signum(thetaPower) * constants.getMaxRotationalSpeedRadiansPerSecond()
        );
    }

    private void updateNetworkTables() {
        Logger.recordOutput("Swerve/Velocity/Rotation", getSelfRelativeVelocity().omegaRadiansPerSecond);
        Logger.recordOutput("Swerve/Velocity/X", getSelfRelativeVelocity().vxMetersPerSecond);
        Logger.recordOutput("Swerve/Velocity/Y", getSelfRelativeVelocity().vyMetersPerSecond);
    }

    @AutoLogOutput(key = "Swerve/CurrentStates")
    private SwerveModuleState[] getModuleStates() {
        final SwerveModuleState[] states = new SwerveModuleState[modulesIO.length];

        for (int i = 0; i < modulesIO.length; i++)
            states[i] = modulesIO[i].getCurrentState();

        return states;
    }

    @AutoLogOutput(key = "Swerve/TargetStates")
    private SwerveModuleState[] getTargetStates() {
        final SwerveModuleState[] states = new SwerveModuleState[modulesIO.length];

        for (int i = 0; i < modulesIO.length; i++)
            states[i] = modulesIO[i].getTargetState();

        return states;
    }

    public boolean isAtPosition(Pose2d pose2d) {
        return isAtXAxisPosition(pose2d.getX()) && isAtYAxisPosition(pose2d.getY()) && isAtAngle(pose2d.getRotation());
    }

    private boolean isAtTranslationPosition(double currentTranslationPosition, double targetTranslationPosition, double currentTranslationVelocity) {
        return Math.abs(currentTranslationPosition - targetTranslationPosition) < SwerveConstants.TRANSLATION_TOLERANCE_METERS &&
                Math.abs(currentTranslationVelocity) < SwerveConstants.TRANSLATION_VELOCITY_TOLERANCE;
    }

    public boolean isAtXAxisPosition(double xAxisPosition) {
        final double currentXAxisVelocity = getFieldRelativeVelocity().vxMetersPerSecond;
        return isAtTranslationPosition(RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose().getX(), xAxisPosition, currentXAxisVelocity);
    }

    public boolean isAtYAxisPosition(double yAxisPosition) {
        final double currentYAxisVelocity = getFieldRelativeVelocity().vyMetersPerSecond;
        return isAtTranslationPosition(RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose().getY(), yAxisPosition, currentYAxisVelocity);
    }

    public boolean isAtAngle(Rotation2d angle) {
        return Math.abs(angle.getDegrees() - RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose().getRotation().getDegrees()) < SwerveConstants.ROTATION_TOLERANCE.getDegrees()
                &&
                Math.abs(getSelfRelativeVelocity().omegaRadiansPerSecond) < SwerveConstants.ROTATION_VELOCITY_TOLERANCE;
    }

    private boolean isStill(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.vxMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND &&
                Math.abs(chassisSpeeds.vyMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND &&
                Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= SwerveConstants.ROTATION_NEUTRAL_DEADBAND;
    }

}