package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.constants.Field;
import frc.robot.constants.LogPaths;
import frc.robot.constants.MathConstants;
import frc.robot.poseestimation.observations.OdometryObservation;
import frc.robot.structures.SuperStructureConstants;
import frc.robot.subsystems.swerve.gyro.GyroInputsAutoLogged;
import frc.robot.subsystems.swerve.gyro.IGyro;
import frc.robot.subsystems.swerve.modules.Modules;
import frc.robot.subsystems.swerve.swervestatehelpers.DriveRelative;
import frc.robot.subsystems.swerve.swervestatehelpers.SwerveStateHelper;
import frc.robot.subsystems.swerve.swervestatehelpers.HeadingControl;
import frc.utils.GBSubsystem;
import frc.utils.cycletime.CycleTimeUtils;
import frc.utils.interpolator.DoubleBilinearInterpolation;
import frc.utils.pathplannerutils.PathPlannerUtils;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.Supplier;


public class Swerve extends GBSubsystem {

	public static final Lock ODOMETRY_LOCK = new ReentrantLock();

	private final SwerveConstants constants;
	private final Modules modules;
	private final IGyro gyro;
	private final GyroInputsAutoLogged gyroInputs;
	private final HeadingStabilizer headingStabilizer;
	private final SwerveCommandsBuilder commandsBuilder;

	private SwerveState currentState;
	private SwerveStateHelper stateHelper;
	private Supplier<Rotation2d> headingSupplier;

	private DoubleBilinearInterpolation discritizationHelper;


	public Swerve(SwerveConstants constants, Modules modules, IGyro gyro) {
		super(constants.logPath());
		this.currentState = new SwerveState(SwerveState.DEFAULT_DRIVE);

		this.constants = constants;
		this.modules = modules;
		this.gyro = gyro;
		this.gyroInputs = new GyroInputsAutoLogged();

		this.headingSupplier = this::getAbsoluteHeading;
		this.headingStabilizer = new HeadingStabilizer(this.constants);
		this.stateHelper = new SwerveStateHelper(Optional::empty, Optional::empty, this);
		this.commandsBuilder = new SwerveCommandsBuilder(this);

		discritizationHelper = new DoubleBilinearInterpolation(constants.discritizationPointsArray());

		updateInputs();
	}

	protected Modules getModules() {
		return modules;
	}

	public SwerveCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public SwerveConstants getConstants() {
		return constants;
	}

	public SwerveStateHelper getStateHelper() {
		return stateHelper;
	}

	@Override
	public String getLogPath() {
		return constants.logPath();
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
		PathPlannerUtils.setLoggingPathToPaths(pose -> Logger.recordOutput(getLogPath() + "CurrentPathToFollow", pose.toArray(new Pose2d[0])));
	}

	public void setStateHelper(SwerveStateHelper swerveStateHelper) {
		this.stateHelper = swerveStateHelper;
	}

	public void setHeadingSupplier(Supplier<Rotation2d> headingSupplier) {
		this.headingSupplier = headingSupplier;
	}

	public void setHeading(Rotation2d heading) {
		gyro.setYaw(heading);
		gyroInputs.gyroYaw = heading;
		headingStabilizer.unlockTarget();
		headingStabilizer.setTargetHeading(heading);
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
		gyroInputs.yawOdometrySamples = new Rotation2d[] {gyroInputs.gyroYaw};
		gyroInputs.timestampOdometrySamples = new double[] {Timer.getFPGATimestamp()};
	}

	private void reportGyroAlerts(GyroInputsAutoLogged gyroInputs) {
		if (!gyroInputs.isConnected) {
			Logger.recordOutput(LogPaths.ALERT_LOG_PATH + constants.gyroLogPath() + "GyroDisconnectedAt", Timer.getFPGATimestamp());
		}
	}

	private void updateInputs() {
		ODOMETRY_LOCK.lock();
		{
			if (Robot.ROBOT_TYPE.isSimulation()) {
				updateGyroSimulation();
			}
			gyro.updateInputs(gyroInputs);
			reportGyroAlerts(gyroInputs);
			Logger.processInputs(constants.gyroLogPath(), gyroInputs);

			modules.logStatus();
		}
		ODOMETRY_LOCK.unlock();
	}

	private void logState() {
		Logger.recordOutput(constants.stateLogPath() + "DriveMode", currentState.getDriveMode());
		Logger.recordOutput(constants.stateLogPath() + "DriveSpeed", currentState.getDriveSpeed());
		Logger.recordOutput(constants.stateLogPath() + "LoopMode", currentState.getLoopMode());
		Logger.recordOutput(constants.stateLogPath() + "RotateAxis", currentState.getRotateAxis());
		Logger.recordOutput(constants.stateLogPath() + "AimAssist", currentState.getAimAssist());
		Logger.recordOutput(constants.stateLogPath() + "HeadingControl", currentState.getHeadingControl());
	}

	private void logFieldRelativeVelocities() {
		ChassisSpeeds fieldRelativeSpeeds = getFieldRelativeVelocity();
		Logger.recordOutput(constants.velocityLogPath() + "Rotation", fieldRelativeSpeeds.omegaRadiansPerSecond);
		Logger.recordOutput(constants.velocityLogPath() + "X", fieldRelativeSpeeds.vxMetersPerSecond);
		Logger.recordOutput(constants.velocityLogPath() + "Y", fieldRelativeSpeeds.vyMetersPerSecond);
		Logger.recordOutput(constants.velocityLogPath() + "Magnitude", SwerveMath.getDriveMagnitude(fieldRelativeSpeeds));
	}

	private void logNumberOfOdometrySamples() {
		Logger.recordOutput(getLogPath() + "OdometrySamples", getNumberOfOdometrySamples());
	}


	public int getNumberOfOdometrySamples() {
		return gyroInputs.timestampOdometrySamples.length;
	}

	public OdometryObservation[] getAllOdometryObservations() {
		int odometrySamples = getNumberOfOdometrySamples();
		double[] timestamps = gyroInputs.timestampOdometrySamples;
		Rotation2d[] gyroHeadings = gyroInputs.yawOdometrySamples;
		SwerveDriveWheelPositions[] swerveWheelPositions = modules.getAllWheelsPositionsSamples();

		OdometryObservation[] odometryObservations = new OdometryObservation[odometrySamples];
		for (int i = 0; i < odometrySamples; i++) {
			odometryObservations[i] = new OdometryObservation(swerveWheelPositions[i], gyroHeadings[i], timestamps[i]);
		}

		return odometryObservations;
	}


	public Rotation2d getAbsoluteHeading() {
		double inputtedHeadingRadians = MathUtil.angleModulus(gyroInputs.gyroYaw.getRadians());
		return Rotation2d.fromRadians(inputtedHeadingRadians);
	}

	public Rotation2d getAllianceRelativeHeading() {
		Rotation2d currentHeading = headingSupplier.get();
		return Field.isFieldConventionAlliance() ? currentHeading : currentHeading.rotateBy(MathConstants.HALF_CIRCLE);
	}

	public ChassisSpeeds getRobotRelativeVelocity() {
		return SwerveConstants.KINEMATICS.toChassisSpeeds(modules.getCurrentStates());
	}

	public ChassisSpeeds getFieldRelativeVelocity() {
		return SwerveMath.robotRelativeToFieldRelativeSpeeds(getRobotRelativeVelocity(), headingSupplier.get());
	}

	private ChassisSpeeds getDriveModeRelativeSpeeds(ChassisSpeeds speeds, SwerveState swerveState) {
		if (swerveState.getDriveMode() == DriveRelative.ROBOT_RELATIVE) {
			return speeds;
		}
		return SwerveMath.fieldRelativeToRobotRelativeSpeeds(speeds, getAllianceRelativeHeading());
	}


	/**
	 * Runs swerve around itself for WheelRadiusCharacterization
	 *
	 * @param rotationsPerSecond - velocity to run the swerve
	 */
	protected void runWheelRadiusCharacterization(Rotation2d rotationsPerSecond) {
		driveByState(new ChassisSpeeds(0, 0, rotationsPerSecond.getRadians()), SwerveState.DEFAULT_DRIVE);
	}


	protected void pidToPose(Pose2d currentPose, Pose2d targetPose) {
		double xVelocityMetersPerSecond = constants.xMetersPIDController().calculate(currentPose.getX(), targetPose.getX());
		double yVelocityMetersPerSecond = constants.yMetersPIDController().calculate(currentPose.getY(), targetPose.getY());
		int direction = Field.isFieldConventionAlliance() ? 1 : -1;
		Rotation2d rotationVelocityPerSecond = Rotation2d.fromDegrees(
			constants.rotationDegreesPIDController().calculate(currentPose.getRotation().getDegrees(), targetPose.getRotation().getDegrees())
		);

		ChassisSpeeds targetFieldRelativeSpeeds = new ChassisSpeeds(
			xVelocityMetersPerSecond * direction,
			yVelocityMetersPerSecond * direction,
			rotationVelocityPerSecond.getRadians()
		);
		driveByState(targetFieldRelativeSpeeds, SwerveState.DEFAULT_DRIVE);
	}

	//@formatter:off
	protected void turnToHeading(Rotation2d targetHeading, SwerveState swerveState) {
		ChassisSpeeds targetSpeeds = new ChassisSpeeds(
			0,
			0,
			Rotation2d.fromDegrees(
					constants.rotationDegreesPIDController().calculate(
							headingSupplier.get().getDegrees(),
							targetHeading.getDegrees()
					)
			).getRadians()
		);
		driveByState(targetSpeeds, swerveState);
	}
	//@formatter:on


	protected void driveByState(double xPower, double yPower, double rotationPower, SwerveState swerveState) {
		ChassisSpeeds speedsFromPowers = SwerveMath.powersToSpeeds(xPower, yPower, rotationPower, constants);
		driveByState(speedsFromPowers, swerveState);
	}

	protected void driveByState(ChassisSpeeds speeds, SwerveState swerveState) {
		this.currentState = swerveState;

		speeds = stateHelper.applyAimAssistOnChassisSpeeds(speeds, swerveState);
		if (SwerveMath.isStill(speeds)) {
			modules.stop();
			return;
		}

		speeds = SwerveMath.factorSpeeds(speeds, swerveState.getDriveSpeed());
		speeds = SwerveMath.applyDeadband(speeds);
		speeds = handleHeadingControl(speeds, swerveState);
		speeds = getDriveModeRelativeSpeeds(speeds, swerveState);
		double fudgeFactor = discritizationHelper
			.getInterpolatedValue(new Translation2d(SwerveMath.getDriveMagnitude(speeds), Math.abs(speeds.omegaRadiansPerSecond)));
		if (fudgeFactor == Double.NaN || fudgeFactor < 1) {
			fudgeFactor = 1;
		}
		speeds = SwerveMath.discretize(speeds, fudgeFactor);
		Logger.recordOutput("fudg factor", fudgeFactor);
		applySpeeds(speeds, swerveState);
	}

	private ChassisSpeeds handleHeadingControl(ChassisSpeeds speeds, SwerveState swerveState) {
		if (swerveState.getHeadingControl() == HeadingControl.NONE) {
			return speeds;
		}

		if (Math.abs(speeds.omegaRadiansPerSecond) > SwerveConstants.ROTATION_NEUTRAL_DEADBAND.getRadians()) {
			headingStabilizer.unlockTarget();
			return speeds;
		}

		headingStabilizer.setTargetHeading(headingSupplier.get());
		headingStabilizer.lockTarget();
		return new ChassisSpeeds(
			speeds.vxMetersPerSecond,
			speeds.vyMetersPerSecond,
			headingStabilizer.calculate(headingSupplier.get()).getRadians()
		);
	}

	private void applySpeeds(ChassisSpeeds speeds, SwerveState swerveState) {
		SwerveModuleState[] swerveModuleStates = SwerveConstants.KINEMATICS
			.toSwerveModuleStates(speeds, stateHelper.getRotationAxis(swerveState.getRotateAxis()));
		setTargetModuleStates(swerveModuleStates, swerveState.getLoopMode().isClosedLoop);
	}

	private void setTargetModuleStates(SwerveModuleState[] moduleStates, boolean isClosedLoop) {
		SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, constants.velocityAt12VoltsMetersPerSecond());
		modules.setTargetStates(moduleStates, isClosedLoop);
	}


	public boolean isAtHeading(Rotation2d targetHeading) {
		double headingDeltaDegrees = Math.abs(targetHeading.minus(headingSupplier.get()).getDegrees());
		boolean isAtHeading = headingDeltaDegrees < SuperStructureConstants.HEADING_TOLERANCE.getDegrees();

		double rotationVelocityRadiansPerSecond = getRobotRelativeVelocity().omegaRadiansPerSecond;
		boolean isStopping = Math.abs(rotationVelocityRadiansPerSecond) < SuperStructureConstants.ROTATION_VELOCITY_TOLERANCE.getRadians();

		return isAtHeading && isStopping;
	}

}
