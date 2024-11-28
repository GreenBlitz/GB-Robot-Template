package frc.robot.subsystems.swerve;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Field;
import frc.robot.constants.MathConstants;
import frc.robot.hardware.empties.EmptyGyro;
import frc.robot.hardware.gyro.maple.MapleGyro;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.poseestimation.observations.OdometryObservation;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.module.Modules;
import frc.robot.subsystems.swerve.module.maple.MapleModule;
import frc.robot.subsystems.swerve.swervestatehelpers.DriveRelative;
import frc.robot.subsystems.swerve.swervestatehelpers.HeadingControl;
import frc.robot.subsystems.swerve.swervestatehelpers.SwerveStateHelper;
import frc.utils.alerts.Alert;
import frc.utils.auto.PathPlannerUtils;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Swerve extends GBSubsystem {

	private final SwerveConstants constants;
	private final Modules modules;
	private final IGyro gyro;
	private final GyroSignals gyroSignals;

	private final HeadingStabilizer headingStabilizer;
	private final SwerveCommandsBuilder commandsBuilder;

	private SwerveState currentState;
	private SwerveStateHelper stateHelper;
	private Supplier<Rotation2d> headingSupplier;
	private Optional<SwerveDriveSimulation> swervePhysicsSimulation;


	public Swerve(SwerveConstants constants, Modules modules, IGyro gyro, GyroSignals gyroSignals) {
		super(constants.logPath());
		this.currentState = new SwerveState(SwerveState.DEFAULT_DRIVE);

		this.constants = constants;
		this.modules = modules;
		this.gyro = gyro;
		this.gyroSignals = gyroSignals;

		this.headingSupplier = this::getGyroAbsoluteYaw;
		this.headingStabilizer = new HeadingStabilizer(this.constants);
		this.stateHelper = new SwerveStateHelper(Optional::empty, Optional::empty, this);
		this.commandsBuilder = new SwerveCommandsBuilder(this);
		this.swervePhysicsSimulation = Optional.empty();
		modules.getCommandsBuilder().withSubsystemsToRequire(this);

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


	public void configPathPlanner(Supplier<Pose2d> currentPoseSupplier, Consumer<Pose2d> resetPoseConsumer, RobotConfig robotConfig) {
		try {
			robotConfig = RobotConfig.fromGUISettings();
		} catch (Exception exception) {
			DriverStation.reportError(exception.getMessage(), exception.getStackTrace());
		} finally {
			PathPlannerUtils.configPathPlanner(
				currentPoseSupplier,
				resetPoseConsumer,
				this::getRobotRelativeVelocity,
				(speeds) -> driveByState(speeds, SwerveState.DEFAULT_PATH_PLANNER),
				constants.ppHolonomicDriveController(),
				robotConfig,
				() -> !Field.isFieldConventionAlliance(),
				this,
				modules
			);
		}
	}

	public void setStateHelper(SwerveStateHelper swerveStateHelper) {
		this.stateHelper = swerveStateHelper;
	}

	public void applyPhysicsSimulation(double robotMassWithBumpersKg, double bumperWidthMeters, double bumperLengthMeters, Pose2d startingPose) {
		if (gyro instanceof MapleGyro && modules.getModule(ModuleUtils.ModulePosition.FRONT_LEFT) instanceof MapleModule) {
			GyroSimulation gyroSimulation = ((MapleGyro) gyro).getGyroSimulation();
			SwerveModuleSimulation[] moduleSimulations = {
				((MapleModule) modules.getModule(ModuleUtils.ModulePosition.FRONT_LEFT)).getModuleSimulation(),
				((MapleModule) modules.getModule(ModuleUtils.ModulePosition.FRONT_RIGHT)).getModuleSimulation(),
				((MapleModule) modules.getModule(ModuleUtils.ModulePosition.BACK_LEFT)).getModuleSimulation(),
				((MapleModule) modules.getModule(ModuleUtils.ModulePosition.BACK_RIGHT)).getModuleSimulation()};

			swervePhysicsSimulation = Optional.of(
				new SwerveDriveSimulation(
					robotMassWithBumpersKg,
					bumperWidthMeters,
					bumperLengthMeters,
					moduleSimulations,
					constants.modulesLocations(),
					gyroSimulation,
					startingPose
				)
			);
			SimulatedArena.getInstance().addDriveTrainSimulation(swervePhysicsSimulation.get());
		} else {
			new Alert(Alert.AlertType.ERROR, constants.logPath() + "Tried to use MAPLE-SIM without MapleGyro and/or MapleModules!!!").report();
			throw new RuntimeException("Tried to use MAPLE-SIM without MapleGyro and/or MapleModules!!!");
		}
	}

	public void setHeadingSupplier(Supplier<Rotation2d> headingSupplier) {
		this.headingSupplier = headingSupplier;
	}

	public void setHeading(Rotation2d heading) {
		gyro.setYaw(heading);
		gyro.updateInputs(gyroSignals.yawSignal());
		headingStabilizer.unlockTarget();
		headingStabilizer.setTargetHeading(heading);
	}

	protected void resetPIDControllers() {
		constants.xMetersPIDController().reset();
		constants.yMetersPIDController().reset();
		constants.rotationDegreesPIDController().reset();
	}


	public void updateStatus() {
		updateInputs();
		logState();
		logFieldRelativeVelocities();
		logNumberOfOdometrySamples();
		logPhysicsSimulationPose();
	}

	private void updateInputs() {
		gyro.updateInputs(gyroSignals.yawSignal());
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

	private void logPhysicsSimulationPose() {
		swervePhysicsSimulation.ifPresent(
			swerveDriveSimulation -> Logger
				.recordOutput(constants.logPath() + "SimulationRobotPosition", swerveDriveSimulation.getSimulatedDriveTrainPose())
		);
	}


	public int getNumberOfOdometrySamples() {
		return Math.min(gyroSignals.yawSignal().asArray().length, modules.getNumberOfOdometrySamples());
	}

	public OdometryObservation[] getAllOdometryObservations() {
		int odometrySamples = getNumberOfOdometrySamples();

		OdometryObservation[] odometryObservations = new OdometryObservation[odometrySamples];
		for (int i = 0; i < odometrySamples; i++) {
			odometryObservations[i] = new OdometryObservation(
				modules.getWheelsPositions(i),
				gyro instanceof EmptyGyro ? null : gyroSignals.yawSignal().asArray()[i],
				gyroSignals.yawSignal().getTimestamps()[i]
			);
		}

		return odometryObservations;
	}

	public Rotation2d getGyroAbsoluteYaw() {
		double inputtedHeadingRadians = MathUtil.angleModulus(gyroSignals.yawSignal().getLatestValue().getRadians());
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

	protected void turnToHeading(Rotation2d targetHeading, SwerveState swerveState) {
		ChassisSpeeds targetSpeeds = new ChassisSpeeds(
			0,
			0,
			Rotation2d
				.fromDegrees(constants.rotationDegreesPIDController().calculate(headingSupplier.get().getDegrees(), targetHeading.getDegrees()))
				.getRadians()
		);
		driveByState(targetSpeeds, swerveState);
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


	public boolean isAtHeading(Rotation2d targetHeading, Rotation2d tolerance, Rotation2d velocityDeadbandAnglesPerSecond) {
		double headingDeltaDegrees = Math.abs(targetHeading.minus(headingSupplier.get()).getDegrees());
		boolean isAtHeading = headingDeltaDegrees < tolerance.getDegrees();

		double rotationVelocityRadiansPerSecond = getRobotRelativeVelocity().omegaRadiansPerSecond;
		boolean isStopping = Math.abs(rotationVelocityRadiansPerSecond) < velocityDeadbandAnglesPerSecond.getRadians();

		return isAtHeading && isStopping;
	}

}
