package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.constants.Field;
import frc.robot.constants.MathConstants;
import frc.robot.hardware.gyro.IGyro;
import frc.robot.poseestimator.observations.OdometryObservation;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.swerve.module.Modules;
import frc.robot.subsystems.swerve.swervestatehelpers.DriveRelative;
import frc.robot.subsystems.swerve.swervestatehelpers.HeadingControl;
import frc.robot.subsystems.swerve.swervestatehelpers.SwerveStateHelper;
import frc.robot.superstructure.Tolerances;
import frc.utils.auto.PathPlannerUtils;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;


public class Swerve extends GBSubsystem {

	private final SwerveConstants constants;
	private final Modules modules;
	private final IGyro gyro;
	private final GyroStuff gyroStuff;

	private final HeadingStabilizer headingStabilizer;
	private final SwerveCommandsBuilder commandsBuilder;

	private SwerveState currentState;
	private SwerveState savedState;
	private SwerveStateHelper stateHelper;
	private Supplier<Rotation2d> headingSupplier;


	public Swerve(SwerveConstants constants, Modules modules, GyroStuff gyroStuff) {
		super(constants.logPath());
		this.savedState = new SwerveState(SwerveState.DEFAULT_DRIVE);
		this.currentState = new SwerveState(savedState);

		this.constants = constants;
		this.modules = modules;
		this.gyro = gyroStuff.gyro();
		this.gyroStuff = gyroStuff;

		this.headingSupplier = this::getGyroAbsoluteYaw;
		this.headingStabilizer = new HeadingStabilizer(this.constants);
		this.stateHelper = new SwerveStateHelper(Optional::empty, Optional::empty, this);
		this.commandsBuilder = new SwerveCommandsBuilder(this);

		updateInputs();
	}

	public Modules getModules() {
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


	public void configPathPlanner(Supplier<Pose2d> currentPoseSupplier, Consumer<Pose2d> resetPoseConsumer) {
		PathPlannerUtils.configPathPlanner(
			currentPoseSupplier,
			resetPoseConsumer,
			this::getRobotRelativeVelocity,
			(speeds) -> driveByState(speeds, SwerveState.DEFAULT_PATH_PLANNER),
			constants.holonomicPathFollowerConfig(),
			() -> !Field.isFieldConventionAlliance(),
			this
		);
	}

	public void setStateHelper(SwerveStateHelper swerveStateHelper) {
		this.stateHelper = swerveStateHelper;
	}

	public void setHeadingSupplier(Supplier<Rotation2d> headingSupplier) {
		this.headingSupplier = headingSupplier;
	}

	public void setHeading(Rotation2d heading) {
		gyro.setYaw(heading);
		gyro.updateSignals(gyroStuff.yawSignal());
		headingStabilizer.unlockTarget();
		headingStabilizer.setTargetHeading(heading);
	}

	protected void resetPIDControllers() {
		constants.xMetersPIDController().reset();
		constants.yMetersPIDController().reset();
		constants.rotationDegreesPIDController().reset();
	}

	public void saveState(SwerveState state) {
		this.savedState = state;
	}


	public void updateStatus() {
		updateInputs();
		logState();
		logFieldRelativeVelocities();
		logNumberOfOdometrySamples();
	}

	private void updateInputs() {
		gyro.updateSignals(gyroStuff.yawSignal());
		modules.updateInputs();
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
		return Math.min(gyroStuff.yawSignal().asArray().length, modules.getNumberOfOdometrySamples());
	}

	public OdometryObservation[] getAllOdometryObservations() {
		int odometrySamples = getNumberOfOdometrySamples();

		OdometryObservation[] odometryObservations = new OdometryObservation[odometrySamples];
		for (int i = 0; i < odometrySamples; i++) {
			odometryObservations[i] = new OdometryObservation(
				modules.getWheelsPositions(i),
				gyroStuff.yawSignal().asArray()[i],
				gyroStuff.yawSignal().getTimestamps()[i]
			);
		}

		return odometryObservations;
	}

	public Rotation2d getGyroAbsoluteYaw() {
		double inputtedHeadingRadians = MathUtil.angleModulus(gyroStuff.yawSignal().getLatestValue().getRadians());
		return Rotation2d.fromRadians(inputtedHeadingRadians);
	}

	public Rotation2d getAbsoluteHeading() {
		double inputtedHeadingRadians = MathUtil.angleModulus(headingSupplier.get().getRadians());
		return Rotation2d.fromRadians(inputtedHeadingRadians);
	}

	public Rotation2d getAllianceRelativeHeading() {
		Rotation2d currentHeading = headingSupplier.get();
		return Field.isFieldConventionAlliance() ? currentHeading : currentHeading.rotateBy(MathConstants.HALF_CIRCLE);
	}

	public ChassisSpeeds getRobotRelativeVelocity() {
		return constants.kinematics().toChassisSpeeds(modules.getCurrentStates());
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


	protected void driveBySavedState(double xPower, double yPower, double rotationPower) {
		ChassisSpeeds speedsFromPowers = SwerveMath.powersToSpeeds(xPower, yPower, rotationPower, constants);
		driveByState(speedsFromPowers, savedState);
	}

	protected void driveByState(double xPower, double yPower, double rotationPower, SwerveState swerveState) {
		ChassisSpeeds speedsFromPowers = SwerveMath.powersToSpeeds(xPower, yPower, rotationPower, constants);
		driveByState(speedsFromPowers, swerveState);
	}

	protected void driveByState(ChassisSpeeds speeds, SwerveState swerveState) {
		this.currentState = swerveState;

		speeds = stateHelper.applyAimAssistOnChassisSpeeds(speeds, swerveState);
		speeds = handleHeadingControl(speeds, swerveState);
		if (SwerveMath.isStill(speeds)) {
			modules.stop();
			return;
		}

		speeds = SwerveMath.factorSpeeds(speeds, swerveState.getDriveSpeed());
		speeds = SwerveMath.applyDeadband(speeds);
		speeds = getDriveModeRelativeSpeeds(speeds, swerveState);
		speeds = SwerveMath.discretize(speeds);

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
		SwerveModuleState[] swerveModuleStates = constants.kinematics()
			.toSwerveModuleStates(speeds, stateHelper.getRotationAxis(swerveState.getRotateAxis()));
		setTargetModuleStates(swerveModuleStates, swerveState.getLoopMode().isClosedLoop);
	}

	private void setTargetModuleStates(SwerveModuleState[] moduleStates, boolean isClosedLoop) {
		SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, constants.velocityAt12VoltsMetersPerSecond());
		modules.setTargetStates(moduleStates, isClosedLoop);
	}


	public boolean isAtHeading(Rotation2d targetHeading) {
		double headingDeltaDegrees = Math.abs(targetHeading.minus(headingSupplier.get()).getDegrees());
		boolean isAtHeading = headingDeltaDegrees < Tolerances.HEADING_TOLERANCE.getDegrees();

		double rotationVelocityRadiansPerSecond = getRobotRelativeVelocity().omegaRadiansPerSecond;
		boolean isStopping = Math.abs(rotationVelocityRadiansPerSecond) < Tolerances.ROTATION_VELOCITY_TOLERANCE.getRadians();

		return isAtHeading && isStopping;
	}

}
