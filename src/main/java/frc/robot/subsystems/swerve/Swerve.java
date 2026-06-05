package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.config.RobotConfig;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.*;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.command2.DeferredCommand;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.sysid.SysIdRoutine;
import frc.constants.MathConstants;
import frc.constants.field.Field;
import frc.joysticks.Axis;
import frc.joysticks.SmartJoystick;
import frc.robot.RobotConstants;
import frc.robot.hardware.empties.EmptyIMU;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.poseestimator.OdometryData;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.swerve.module.Modules;
import frc.robot.subsystems.swerve.states.DriveRelative;
import frc.robot.subsystems.swerve.states.LoopMode;
import frc.robot.subsystems.swerve.states.SwerveStateHandler;
import frc.robot.subsystems.swerve.states.heading.HeadingControl;
import frc.robot.subsystems.swerve.states.heading.HeadingStabilizer;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.utils.TimedValue;
import frc.utils.auto.PathPlannerUtil;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Swerve extends GBSubsystem {

	private final SwerveConstants constants;
	private final double driveRadiusMeters;
	private final Modules modules;
	private final IIMU imu;
	private final IMUSignals imuSignals;

	private final SwerveDriveKinematics kinematics;
	private final HeadingStabilizer headingStabilizer;
	private final SwerveCommandsBuilder commandsBuilder;
	private final SwerveStateHandler stateHandler;

	private SwerveState currentState;
	private Supplier<Rotation2d> headingSupplier;
	private ChassisPowers driversPowerInputs;

	public Swerve(SwerveConstants constants, Modules modules, IIMU imu, IMUSignals imuSignals) {
		super(constants.logPath());
		this.currentState = new SwerveState(SwerveState.DEFAULT_DRIVE);
		this.driversPowerInputs = new ChassisPowers();

		this.constants = constants;
		this.driveRadiusMeters = SwerveMath.calculateDriveRadiusMeters(modules.getModulePositionsFromCenterMeters());
		this.modules = modules;
		this.imu = imu;
		this.imuSignals = imuSignals;

		this.kinematics = new SwerveDriveKinematics(modules.getModulePositionsFromCenterMeters());
		this.headingSupplier = () -> getGyroAbsoluteYaw().getValue();
		this.headingStabilizer = new HeadingStabilizer(this.constants);
		this.stateHandler = new SwerveStateHandler(this);
		this.commandsBuilder = new SwerveCommandsBuilder(this);

		update();
		setDefaultCommand(commandsBuilder.driveByDriversInputs(SwerveState.DEFAULT_DRIVE));
	}

	public String getLogPath() {
		return constants.logPath();
	}

	public Modules getModules() {
		return modules;
	}

	public SwerveDriveKinematics getKinematics() {
		return kinematics;
	}

	public SwerveCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public SwerveConstants getConstants() {
		return constants;
	}

	public SwerveStateHandler getStateHandler() {
		return stateHandler;
	}

	public Rotation3d getAngularVelocityFromIMURotation2dPerSecond() {
		return imuSignals.getAngularVelocity();
	}

	public Rotation3d getOrientationFromIMU() {
		return imuSignals.getOrientation();
	}

	public Translation3d getAccelerationFromIMUMetersPerSecondSquared() {
		return imuSignals.getAccelerationEarthGravitationalAcceleration()
			.times(RobotConstants.GRAVITATIONAL_ACCELERATION_METERS_PER_SECOND_SQUARED_ISRAEL);
	}


	public void configPathPlanner(Supplier<Pose2d> currentPoseSupplier, Consumer<Pose2d> resetPoseConsumer, RobotConfig robotConfig) {
		PathPlannerUtil.configPathPlanner(
			currentPoseSupplier,
			resetPoseConsumer,
			this::getRobotRelativeVelocity,
			(velocities) -> driveByState(velocities, SwerveState.DEFAULT_PATH_PLANNER),
			constants.pathPlannerHolonomicDriveController(),
			robotConfig,
			() -> !Field.isFieldConventionAlliance(),
			this
		);
	}

	public void setHeadingSupplier(Supplier<Rotation2d> headingSupplier) {
		this.headingSupplier = headingSupplier;
	}

	public void setDriversPowerInputs(ChassisPowers powers) {
		this.driversPowerInputs = powers;
	}

	public void setHeading(Rotation2d heading) {
		imu.setYaw(heading);
		updateIMU();
		headingStabilizer.unlockTarget();
		headingStabilizer.setTargetHeading(heading);
	}

	protected void resetPIDControllers() {
		constants.xMetersPIDController().reset();
		constants.yMetersPIDController().reset();
		constants.rotationDegreesPIDController().reset();
	}

	private void updateIMU() {
		imu.updateInputs(
			imuSignals.pitchSignal(),
			imuSignals.rollSignal(),
			imuSignals.yawSignal(),
			imuSignals.rollAngularVelocitySignal(),
			imuSignals.pitchAngularVelocitySignal(),
			imuSignals.yawAngularVelocitySignal(),
			imuSignals.xAccelerationSignalEarthGravitationalAcceleration(),
			imuSignals.yAccelerationSignalEarthGravitationalAcceleration(),
			imuSignals.zAccelerationSignalEarthGravitationalAcceleration()
		);
	}

	public void update() {
		updateIMU();
		modules.updateInputs();

		currentState.log(constants.stateLogPath());

		ChassisVelocities allianceRelativeVelocities = getAllianceRelativeVelocity();
		Logger.recordOutput(constants.velocityLogPath() + "/Rotation", allianceRelativeVelocities.omega);
		Logger.recordOutput(constants.velocityLogPath() + "/X", allianceRelativeVelocities.vx);
		Logger.recordOutput(constants.velocityLogPath() + "/Y", allianceRelativeVelocities.vy);
		Logger.recordOutput(constants.velocityLogPath() + "/Magnitude", SwerveMath.getDriveMagnitude(allianceRelativeVelocities));

		Logger.recordOutput(getLogPath() + "/OdometrySamples", getNumberOfOdometrySamples());

		Logger.recordOutput(getLogPath() + "/IMU/Acceleration", getAccelerationFromIMUMetersPerSecondSquared());

		Logger.recordOutput(getLogPath() + "/isCollisionDetected", isCollisionDetected());
	}


	public int getNumberOfOdometrySamples() {
		return Math.min(imuSignals.yawSignal().asArray().length, modules.getNumberOfOdometrySamples());
	}

	public OdometryData[] getAllOdometryData() {
		OdometryData[] odometryData = new OdometryData[getNumberOfOdometrySamples()];

		for (int i = 0; i < odometryData.length; i++) {
			odometryData[i] = new OdometryData(
				imuSignals.yawSignal().getTimestamps()[i],
				modules.getWheelPositions(i),
				imu instanceof EmptyIMU ? Optional.empty() : Optional.of(imuSignals.yawSignal().asArray()[i]),
				imu instanceof EmptyIMU ? Optional.empty() : Optional.of(getIMUAcceleration())
			);
		}

		return odometryData;
	}

	public double getDriveRadiusMeters() {
		return driveRadiusMeters;
	}

	public TimedValue<Rotation2d> getGyroAbsoluteYaw() {
		TimedValue<Rotation2d> latestGyroYaw = imuSignals.yawSignal().getLatestTimedValue();
		Rotation2d latestGyroAbsoluteYaw = Rotation2d.fromRadians(MathUtil.angleModulus(latestGyroYaw.getValue().getRadians()));
		return new TimedValue<>(latestGyroAbsoluteYaw, latestGyroYaw.getTimestamp());
	}

	public Rotation2d getAbsoluteHeading() {
		double inputtedHeadingRadians = MathUtil.angleModulus(headingSupplier.get().getRadians());
		return Rotation2d.fromRadians(inputtedHeadingRadians);
	}

	public Rotation2d getAllianceRelativeHeading() {
		Rotation2d currentHeading = headingSupplier.get();
		return Field.isFieldConventionAlliance() ? currentHeading : currentHeading.rotateBy(MathConstants.HALF_CIRCLE);
	}

	public ChassisVelocities getRobotRelativeVelocity() {
		return kinematics.toChassisVelocities(modules.getCurrentVelocities());
	}

	public ChassisVelocities getAllianceRelativeVelocity() {
		return SwerveMath.robotToAllianceRelativeVelocities(getRobotRelativeVelocity(), getAllianceRelativeHeading());
	}

	private ChassisVelocities getDriveModeRelativeVelocities(ChassisVelocities velocities, SwerveState swerveState) {
		if (swerveState.getDriveRelative() == DriveRelative.ROBOT_RELATIVE) {
			return velocities;
		}
		return SwerveMath.allianceToRobotRelativeVelocities(velocities, getAllianceRelativeHeading());
	}

	public double getIMUAcceleration() {
		return imuSignals.getAccelerationEarthGravitationalAcceleration().getNorm();
	}


	protected void moveToPoseByPID(Pose2d currentPose, Pose2d targetPose) {
		double xVelocityMetersPerSecond = constants.xMetersPIDController().calculate(currentPose.getX(), targetPose.getX());
		double yVelocityMetersPerSecond = constants.yMetersPIDController().calculate(currentPose.getY(), targetPose.getY());
		int direction = Field.isFieldConventionAlliance() ? 1 : -1;
		Rotation2d rotationVelocityPerSecond = Rotation2d.fromDegrees(
			constants.rotationDegreesPIDController().calculate(currentPose.getRotation().getDegrees(), targetPose.getRotation().getDegrees())
		);

		ChassisVelocities targetAllianceRelativeVelocities = new ChassisVelocities(
			xVelocityMetersPerSecond * direction,
			yVelocityMetersPerSecond * direction,
			rotationVelocityPerSecond.getRadians()
		);
		driveByState(targetAllianceRelativeVelocities, SwerveState.DEFAULT_DRIVE);
	}

	protected void turnToHeading(Rotation2d targetHeading, SwerveState swerveState) {
		ChassisVelocities targetVelocities = new ChassisVelocities(
			0,
			0,
			Rotation2d
				.fromDegrees(constants.rotationDegreesPIDController().calculate(headingSupplier.get().getDegrees(), targetHeading.getDegrees()))
				.getRadians()
		);
		driveByState(targetVelocities, swerveState);
	}

	protected void driveByDriversTargetsPowers(SwerveState swerveState) {
		driveByState(driversPowerInputs, swerveState);
	}

	protected void driveByState(ChassisPowers powers, SwerveState swerveState) {
		ChassisVelocities velocitiesFromPowers = SwerveMath.powersToVelocities(powers, constants);
		driveByState(velocitiesFromPowers, swerveState);
	}

	protected void driveByState(ChassisVelocities velocities, SwerveState swerveState) {
		this.currentState = swerveState;

		velocities = stateHandler.applyAimAssistOnChassisVelocities(velocities, swerveState);
		velocities = handleHeadingControl(velocities, swerveState);
		if (SwerveMath.isStill(velocities, SwerveConstants.DEADBANDS)) {
			modules.stop();
			return;
		}

		velocities = SwerveMath.factorVelocities(velocities, swerveState.getDriveSpeed());
		velocities = SwerveMath.applyDeadband(velocities, SwerveConstants.DEADBANDS);
		velocities = getDriveModeRelativeVelocities(velocities, swerveState);
		velocities = SwerveMath.discretize(velocities);

		applyVelocities(velocities, swerveState);
	}

	private ChassisVelocities handleHeadingControl(ChassisVelocities velocities, SwerveState swerveState) {
		if (swerveState.getHeadingControl() == HeadingControl.NONE) {
			return velocities;
		}

		if (Math.abs(velocities.omega) > SwerveConstants.DEADBANDS.getRotation().getRadians()) {
			headingStabilizer.unlockTarget();
			return velocities;
		}

		headingStabilizer.setTargetHeading(headingSupplier.get());
		headingStabilizer.lockTarget();
		return new ChassisVelocities(
			velocities.vx,
			velocities.vy,
			headingStabilizer.calculatePIDOutput(headingSupplier.get()).getRadians()
		);
	}

	private void applyVelocities(ChassisVelocities velocities, SwerveState swerveState) {
		SwerveModuleVelocity[] swerveModuleVelocities = kinematics
			.toSwerveModuleVelocities(velocities, stateHandler.getRotationAxis(swerveState.getRotateAxis()));
		setTargetModuleVelocities(swerveModuleVelocities, swerveState.getLoopMode().isClosedLoop());
	}

	private void setTargetModuleVelocities(SwerveModuleVelocity[] moduleVelocities, boolean isClosedLoop) {
        SwerveModuleVelocity[] desaturatedModuleVelocities = SwerveDriveKinematics.desaturateWheelVelocities(moduleVelocities, constants.velocityAt12VoltsMetersPerSecond());
		modules.setTargetVelocities(desaturatedModuleVelocities, isClosedLoop);
	}


	public boolean isAtHeading(Rotation2d targetHeading, Rotation2d tolerance, Rotation2d velocityDeadbandAnglesPerSecond) {
		double headingDeltaDegrees = Math.abs(targetHeading.minus(headingSupplier.get()).getDegrees());
		boolean isAtHeading = headingDeltaDegrees < tolerance.getDegrees();

		double rotationVelocityRadiansPerSecond = getRobotRelativeVelocity().omega;
		boolean isStopping = Math.abs(rotationVelocityRadiansPerSecond) < velocityDeadbandAnglesPerSecond.getRadians();

		return isAtHeading && isStopping;
	}

	public boolean isCollisionDetected() {
		return imuSignals.getAccelerationEarthGravitationalAcceleration().toTranslation2d().getNorm() > SwerveConstants.MIN_COLLISION_G_FORCE;
	}

	public void applyCalibrationBindings(SmartJoystick joystick, Supplier<Pose2d> robotPoseSupplier) {
		joystick.START.onTrue(new InstantCommand(() -> commandsBuilder.setIsSubsystemRunningIndependently(true)));
		joystick.BACK.onTrue(new InstantCommand(() -> commandsBuilder.setIsSubsystemRunningIndependently(false)));

		// Calibrate steer ks with phoenix tuner x
		// Calibrate steer pid with phoenix tuner x

		// Let it rotate some rotations then output will be in log under Calibrations/.
		joystick.POV_DOWN.whileTrue(getCommandsBuilder().wheelRadiusCalibration());

		// ROBOT RELATIVE DRIVE - FOR GYRO TEST
		joystick.POV_UP
			.whileTrue(commandsBuilder.driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withDriveRelative(DriveRelative.ROBOT_RELATIVE)));

		// Test the swerve returns real velocities (measure distance and time in real life and compare to swerve velocity logs).
		// REMEMBER after drive calibrations use these for pid testing - Remove OPEN LOOP for that
		ChassisPowers slowCalibrationPowers = new ChassisPowers();
		slowCalibrationPowers.xPower = 0.2;
		joystick.POV_LEFT
			.whileTrue(getCommandsBuilder().driveByState(() -> slowCalibrationPowers, SwerveState.DEFAULT_DRIVE.withLoopMode(LoopMode.OPEN)));
		ChassisPowers fastCalibrationPowers = new ChassisPowers();
		fastCalibrationPowers.xPower = 0.5;
		joystick.POV_RIGHT
			.whileTrue(getCommandsBuilder().driveByState(() -> fastCalibrationPowers, SwerveState.DEFAULT_DRIVE.withLoopMode(LoopMode.OPEN)));

		// The sysid outputs will be logged to the "CTRE Signal Logger".
		// Use phoenix tuner x to extract the position, velocity, motorVoltage, state signals into wpilog.
		// Then enter the wpilog into wpilib sysid app and make sure you enter all info in the correct places.
		// (see wpilib sysid in google)
		joystick.Y.whileTrue(getCommandsBuilder().driveCalibration(true, SysIdRoutine.Direction.kForward));
		joystick.A.whileTrue(getCommandsBuilder().driveCalibration(true, SysIdRoutine.Direction.kReverse));
		joystick.X.whileTrue(getCommandsBuilder().driveCalibration(false, SysIdRoutine.Direction.kForward));
		joystick.B.whileTrue(getCommandsBuilder().driveCalibration(false, SysIdRoutine.Direction.kReverse));
		// MAKE SURE TO PRESS IT ON THE END OF THE SYSID ROUTINE SO YOU CAN READ THE DATA FROM SIGNAL LOGGER.
		joystick.L3.onTrue(new InstantCommand(SignalLogger::stop));

		// Remember to test the drive pid ff calib with the POVS commands

		// Rotational pid tests
		joystick.R1.whileTrue(getCommandsBuilder().turnToHeading(MathConstants.HALF_CIRCLE));
		joystick.L1.whileTrue(getCommandsBuilder().turnToHeading(new Rotation2d()));

		// Translation pid tests
		joystick.getAxisAsButton(Axis.LEFT_TRIGGER)
			.onTrue(
				new DeferredCommand(
					() -> getCommandsBuilder()
						.moveToPoseByPID(robotPoseSupplier, robotPoseSupplier.get().plus(new Transform2d(1, 1, new Rotation2d()))),
					Set.of(this)
				)
			);
		joystick.getAxisAsButton(Axis.RIGHT_TRIGGER)
			.onTrue(
				new DeferredCommand(
					() -> getCommandsBuilder()
						.moveToPoseByPID(robotPoseSupplier, robotPoseSupplier.get().plus(new Transform2d(-1, -1, new Rotation2d()))),
					Set.of(this)
				)
			);
	}

}
