package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.MathConstants;
import frc.robot.subsystems.swerve.swerveinterface.ISwerve;
import frc.robot.subsystems.swerve.swerveinterface.SwerveFactory;
import frc.robot.subsystems.swerve.swerveinterface.SwerveInputsAutoLogged;
import frc.utils.GBSubsystem;
import frc.utils.allianceutils.AllianceUtils;
import frc.utils.roborioutils.RoborioUtils;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class Swerve extends GBSubsystem {

    public final Lock odometryLock;

    private final SwerveInputsAutoLogged swerveInputs;

    private final ISwerve swerve;

    private final Module[] modules;

    public Swerve() {
        setName("Swerve");

        swerve = SwerveFactory.createSwerve();
        modules = getModules();

        swerveInputs = new SwerveInputsAutoLogged();
        odometryLock = new ReentrantLock();

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

    public void resetByEncoder() {
        for (Module module : getModules()) {
            module.resetByEncoder();
        }
    }

    @Override
    public void periodic() {
        odometryLock.lock();
        swerve.updateInputs(swerveInputs);
        Logger.processInputs(this.getName(), swerveInputs);

        for (Module module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        updateNetworkTables();
    }

    private void updateNetworkTables() {
        Logger.recordOutput("Swerve/Velocity/Rotation", getSelfRelativeVelocity().omegaRadiansPerSecond);
        Logger.recordOutput("Swerve/Velocity/X", getSelfRelativeVelocity().vxMetersPerSecond);
        Logger.recordOutput("Swerve/Velocity/Y", getSelfRelativeVelocity().vyMetersPerSecond);
    }

    public ChassisSpeeds getSelfRelativeVelocity() {
        return SwerveConstants.KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    @AutoLogOutput(key = "Swerve/CurrentStates")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getCurrentState();
        }

        return states;
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(swerveInputs.gyroPitchDegrees);
    }

    public Translation3d getGyroAcceleration() {
        return new Translation3d(swerveInputs.accelerationX, swerveInputs.accelerationY, swerveInputs.accelerationZ);
    }

    public void setBrake(boolean brake) {
        for (Module currentModule : modules) {
            currentModule.setBrake(brake);
        }
    }

    public ChassisSpeeds getFieldRelativeVelocity() {
        return new ChassisSpeeds();
        //        return ChassisSpeeds.fromFieldRelativeSpeeds(getSelfRelativeVelocity(),
        // RobotContainer.POSE_ESTIMATOR.getCurrentPose().toAlliancePose().getRotation());
    }

    protected void pidToPose(Pose2d targetPose) {
        //        final Pose2d currentPose = RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose();
        //        final ChassisSpeeds targetFieldRelativeSpeeds = new ChassisSpeeds(
        //                SwerveConstants.TRANSLATION_PID_CONTROLLER.calculate(currentPose.getX(), targetPose.getX()),
        //                SwerveConstants.TRANSLATION_PID_CONTROLLER.calculate(currentPose.getY(), targetPose.getY()),
        //                calculateProfiledAngleSpeedToTargetAngle(targetPose.getRotation())
        //        );
        //        selfRelativeDrive(fieldRelativeSpeedsToSelfRelativeSpeeds(targetFieldRelativeSpeeds));
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
        //
        // SwerveConstants.PROFILED_ROTATION_PID_CONTROLLER.reset(RobotContainer.POSE_ESTIMATOR.getCurrentPose()
        // .toBlueAlliancePose().getRotation().getDegrees());
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

    private ChassisSpeeds selfRelativeSpeedsFromFieldRelativePowers(double xPower, double yPower, double thetaPower) {
        ChassisSpeeds fieldRelativeSpeeds = powersToSpeeds(xPower, yPower, thetaPower);
        return fieldRelativeSpeedsToSelfRelativeSpeeds(fieldRelativeSpeeds);
    }

    private double calculateProfiledAngleSpeedToTargetAngle(Rotation2d targetAngle) {
        return 0;
        //        final Rotation2d currentAngle =
        // RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose().getRotation();
        //        return
        // Units.degreesToRadians(SwerveConstants.PROFILED_ROTATION_PID_CONTROLLER.calculate(currentAngle.getDegrees(),
        // targetAngle.getDegrees()));
    }

    private void selfRelativeDrive(ChassisSpeeds chassisSpeeds) {
        chassisSpeeds = discretize(chassisSpeeds);
        if (isStill(chassisSpeeds)) {
            stop();
            return;
        }

        final SwerveModuleState[] swerveModuleStates = SwerveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        setTargetModuleStates(swerveModuleStates);
    }

    private ChassisSpeeds powersToSpeeds(double xPower, double yPower, double thetaPower) {
        return new ChassisSpeeds(
                xPower * SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
                yPower * SwerveConstants.MAX_SPEED_METERS_PER_SECOND,
                Math.pow(thetaPower, 2) * Math.signum(thetaPower) * SwerveConstants.MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND
        );
    }

    private ChassisSpeeds fieldRelativeSpeedsToSelfRelativeSpeeds(ChassisSpeeds fieldRelativeSpeeds) {
        //        final Rotation2d currentAngle =
        // RobotContainer.POSE_ESTIMATOR.getCurrentPose().toAlliancePose().getRotation();
        Rotation2d currentAngle = getHeading();
        return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, currentAngle);
    }

    /**
     * When the robot drives while rotating it skews a bit to the side.
     * This should fix the chassis speeds, so they won't make the robot skew while rotating.
     *
     * @param chassisSpeeds the chassis speeds to fix skewing for
     * @return the fixed speeds
     */
    private ChassisSpeeds discretize(ChassisSpeeds chassisSpeeds) {
        return ChassisSpeeds.discretize(chassisSpeeds, RoborioUtils.getCurrentRoborioCycle());
    }

    public boolean isStill(ChassisSpeeds chassisSpeeds) {
        return Math.abs(chassisSpeeds.vxMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND && Math.abs(chassisSpeeds.vyMetersPerSecond) <= SwerveConstants.DRIVE_NEUTRAL_DEADBAND && Math.abs(
                chassisSpeeds.omegaRadiansPerSecond) <= SwerveConstants.ROTATION_NEUTRAL_DEADBAND;
    }

    public void stop() {
        for (Module currentModule : modules) {
            currentModule.stop();
        }
    }

    private void setTargetModuleStates(SwerveModuleState[] swerveModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.MAX_SPEED_METERS_PER_SECOND);
        for (int i = 0; i < modules.length; i++) {
            modules[i].setTargetState(swerveModuleStates[i]);
        }
    }

    public Rotation2d getHeading() {
        double inputtedHeading = MathUtil.inputModulus(
                swerveInputs.gyroYawDegrees,
                -MathConstants.HALF_CIRCLE.getDegrees(),
                MathConstants.HALF_CIRCLE.getDegrees()
        );
        return Rotation2d.fromDegrees(inputtedHeading);
    }

    public void setHeading(Rotation2d heading) {
        swerve.setHeading(heading);
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

    @AutoLogOutput(key = "Swerve/TargetStates")
    private SwerveModuleState[] getTargetStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];

        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getTargetState();
        }

        return states;
    }

    public boolean isAtPosition(Pose2d pose2d) {
        return isAtXAxisPosition(pose2d.getX()) && isAtYAxisPosition(pose2d.getY()) && isAtAngle(pose2d.getRotation());
    }

    public boolean isAtXAxisPosition(double xAxisPosition) {
        return true;
        //        final double currentXAxisVelocity = getFieldRelativeVelocity().vxMetersPerSecond;
        //        return
        // isAtTranslationPosition(RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose().getX(),
        // xAxisPosition, currentXAxisVelocity);
    }

    public boolean isAtYAxisPosition(double yAxisPosition) {
        return true;
        //        final double currentYAxisVelocity = getFieldRelativeVelocity().vyMetersPerSecond;
        //        return
        // isAtTranslationPosition(RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose().getY(),
        // yAxisPosition, currentYAxisVelocity);
    }

    public boolean isAtAngle(Rotation2d angle) {
        return true;
        //        return Math.abs(angle.getDegrees() -
        // RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose().getRotation().getDegrees()) <
        // SwerveConstants.ROTATION_TOLERANCE.getDegrees()
        //                &&
        //                Math.abs(getSelfRelativeVelocity().omegaRadiansPerSecond) <
        // SwerveConstants.ROTATION_VELOCITY_TOLERANCE;
    }

    private boolean isAtTranslationPosition(
            double currentTranslationPosition, double targetTranslationPosition, double currentTranslationVelocity
    ) {
        return Math.abs(currentTranslationPosition - targetTranslationPosition) < SwerveConstants.TRANSLATION_TOLERANCE_METERS && Math.abs(
                currentTranslationVelocity) < SwerveConstants.TRANSLATION_VELOCITY_TOLERANCE;
    }

}
