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
import edu.wpi.first.math.util.Units;
import frc.robot.constants.MathConstants;
import frc.robot.subsystems.swerve.gyro.GyroFactory;
import frc.robot.subsystems.swerve.gyro.GyroInputsAutoLogged;
import frc.robot.subsystems.swerve.gyro.IGyro;
import frc.robot.subsystems.swerve.modules.Module;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.utils.DriverStationUtils;
import frc.utils.GBSubsystem;
import frc.utils.allianceutils.AllianceUtils;
import frc.utils.roborioutils.RoborioUtils;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Swerve extends GBSubsystem {

    public final Lock ODOMETRY_LOCK;

    private final GyroInputsAutoLogged gyroInputs;

    private final IGyro gyro;

    private final Module[] modules;


    public Swerve() {
        setName("Swerve");

        gyro = GyroFactory.createSwerve();
        modules = getModules();

        gyroInputs = new GyroInputsAutoLogged();
        ODOMETRY_LOCK = new ReentrantLock();

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
        //        AutoBuilder.configureHolonomic(
        //                () -> RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose(),
        //                (pose) -> RobotContainer.POSE_ESTIMATOR.resetPose(AlliancePose2d.fromBlueAlliancePose(pose)),
        //                this::getSelfRelativeVelocity,
        //                this::selfRelativeDrive,
        //                SwerveConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG,
        //                () -> !DriverStationUtils.isBlueAlliance(),
        //                this
        //        );
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
        // Todo
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

    protected void initializeDrive(boolean closedLoop) {
        setClosedLoop(closedLoop);
        resetRotationController();
    }

    protected void setClosedLoop(boolean closedLoop) {
        for (Module currentModule : modules) {
            currentModule.setDriveMotorClosedLoop(closedLoop);
        }
    }

    protected void resetRotationController() {
        SwerveConstants.PROFILED_ROTATION_PID_CONTROLLER.reset(
                POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose().getRotation().getDegrees()
        );
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
                RobotContainer.POSE_ESTIMATOR.getCurrentPose().toAlliancePose().getRotation()
        );
    }

    
    public void runWheelRadiusCharacterization(double omegaRadsPerSec) {
        selfRelativeDrive(new ChassisSpeeds(0, 0, omegaRadsPerSec));
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

    private SwerveDriveWheelPositions getSwerveWheelPositions(int odometryUpdateIndex) {
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

    private double calculateProfiledAngleSpeedToTargetAngle(Rotation2d targetAngle) {
        Rotation2d currentAngle = POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose().getRotation();
        return Units.degreesToRadians(SwerveConstants.PROFILED_ROTATION_PID_CONTROLLER.calculate(
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
                xPower * SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
                yPower * SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
                Math.pow(thetaPower, 2) * Math.signum(thetaPower) * SwerveConstants.MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND
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
        //Todo - check what about when you in 180 and want to be in -180
        double angleDifference = Math.abs(targetAngle.minus(POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose().getRotation()));
        boolean isAtAngle = angleDifference < SwerveConstants.ROTATION_TOLERANCE.getDegrees();
        boolean isStopping = Math.abs(getSelfRelativeVelocity().omegaRadiansPerSecond) < SwerveConstants.ROTATION_VELOCITY_TOLERANCE;
        return isAtAngle && isStopping;
    }

    public boolean isAtPosition(Pose2d targetPose) {
        return isAtXAxisPosition(targetPose.getX()) && isAtYAxisPosition(targetPose.getY()) && isAtAngle(targetPose.getRotation());
    }

}
