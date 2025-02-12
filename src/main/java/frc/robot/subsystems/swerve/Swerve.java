package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.constants.MathConstants;
import frc.constants.field.Field;
import frc.joysticks.Axis;
import frc.joysticks.SmartJoystick;
import frc.robot.hardware.empties.EmptyGyro;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.poseestimator.OdometryData;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.swerve.module.Modules;
import frc.robot.subsystems.swerve.states.DriveRelative;
import frc.robot.subsystems.swerve.states.LoopMode;
import frc.robot.subsystems.swerve.states.SwerveStateHandler;
import frc.robot.subsystems.swerve.states.heading.HeadingControl;
import frc.robot.subsystems.swerve.states.heading.HeadingStabilizer;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.calibration.swervecalibration.maxvelocityacceleration.VelocityType;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Swerve extends GBSubsystem {

	private final SwerveConstants constants;
	private final double driveRadiusMeters;
	private final Modules modules;
	private final IGyro gyro;
	private final GyroSignals gyroSignals;

	private final SwerveDriveKinematics kinematics;
	private final HeadingStabilizer headingStabilizer;
	private final SwerveCommandsBuilder commandsBuilder;
	private final SwerveStateHandler stateHandler;

	private SwerveState currentState;
	private Supplier<Rotation2d> headingSupplier;
	private ChassisPowers driversPowerInputs;

	public Swerve(SwerveConstants constants, Modules modules, IGyro gyro, GyroSignals gyroSignals) {
		super(constants.logPath());
		this.currentState = new SwerveState(SwerveState.DEFAULT_DRIVE);
		this.driversPowerInputs = new ChassisPowers(0, 0, 0);

		this.constants = constants;
		this.driveRadiusMeters = SwerveMath.calculateDriveRadiusMeters(modules.getModulePositionsFromCenterMeters());
		this.modules = modules;
		this.gyro = gyro;
		this.gyroSignals = gyroSignals;

		this.kinematics = new SwerveDriveKinematics(modules.getModulePositionsFromCenterMeters());
		this.headingSupplier = this::getGyroAbsoluteYaw;
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


	public void configPathPlanner(Supplier<Pose2d> currentPoseSupplier, Consumer<Pose2d> resetPoseConsumer, RobotConfig robotConfig) {
		PathPlannerUtil.configPathPlanner(
			currentPoseSupplier,
			resetPoseConsumer,
			this::getRobotRelativeVelocity,
			(speeds) -> driveByState(speeds, SwerveState.DEFAULT_PATH_PLANNER),
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


	public void update() {
		gyro.updateInputs(gyroSignals.yawSignal());
		modules.updateInputs();

		currentState.log(constants.stateLogPath());

		ChassisSpeeds allianceRelativeSpeeds = getAllianceRelativeVelocity();
		Logger.recordOutput(constants.velocityLogPath() + "/Rotation", allianceRelativeSpeeds.omegaRadiansPerSecond);
		Logger.recordOutput(constants.velocityLogPath() + "/X", allianceRelativeSpeeds.vxMetersPerSecond);
		Logger.recordOutput(constants.velocityLogPath() + "/Y", allianceRelativeSpeeds.vyMetersPerSecond);
		Logger.recordOutput(constants.velocityLogPath() + "/Magnitude", SwerveMath.getDriveMagnitude(allianceRelativeSpeeds));

		Logger.recordOutput(getLogPath() + "/OdometrySamples", getNumberOfOdometrySamples());
	}


	public int getNumberOfOdometrySamples() {
		return Math.min(gyroSignals.yawSignal().asArray().length, modules.getNumberOfOdometrySamples());
	}

	public OdometryData[] getAllOdometryData() {
		OdometryData[] odometryData = new OdometryData[getNumberOfOdometrySamples()];

		for (int i = 0; i < odometryData.length; i++) {
			odometryData[i] = new OdometryData(
				modules.getWheelPositions(i),
				gyro instanceof EmptyGyro ? Optional.empty() : Optional.of(gyroSignals.yawSignal().asArray()[i]),
				gyroSignals.yawSignal().getTimestamps()[i]
			);
		}

		return odometryData;
	}

	public double getDriveRadiusMeters() {
		return driveRadiusMeters;
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
		return kinematics.toChassisSpeeds(modules.getCurrentStates());
	}

	public ChassisSpeeds getAllianceRelativeVelocity() {
		return SwerveMath.robotToAllianceRelativeSpeeds(getRobotRelativeVelocity(), getAllianceRelativeHeading());
	}

	private ChassisSpeeds getDriveModeRelativeSpeeds(ChassisSpeeds speeds, SwerveState swerveState) {
		if (swerveState.getDriveRelative() == DriveRelative.ROBOT_RELATIVE) {
			return speeds;
		}
		return SwerveMath.allianceToRobotRelativeSpeeds(speeds, getAllianceRelativeHeading());
	}


	protected void moveToPoseByPID(Pose2d currentPose, Pose2d targetPose) {
		double xVelocityMetersPerSecond = constants.xMetersPIDController().calculate(currentPose.getX(), targetPose.getX());
		double yVelocityMetersPerSecond = constants.yMetersPIDController().calculate(currentPose.getY(), targetPose.getY());
		int direction = Field.isFieldConventionAlliance() ? 1 : -1;
		Rotation2d rotationVelocityPerSecond = Rotation2d.fromDegrees(
			constants.rotationDegreesPIDController().calculate(currentPose.getRotation().getDegrees(), targetPose.getRotation().getDegrees())
		);

		ChassisSpeeds targetAllianceRelativeSpeeds = new ChassisSpeeds(
			xVelocityMetersPerSecond * direction,
			yVelocityMetersPerSecond * direction,
			rotationVelocityPerSecond.getRadians()
		);
		driveByState(targetAllianceRelativeSpeeds, SwerveState.DEFAULT_DRIVE);
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

	protected void driveByDriversTargetsPowers(SwerveState swerveState) {
		driveByState(driversPowerInputs, swerveState);
	}

	protected void driveByState(ChassisPowers powers, SwerveState swerveState) {
		ChassisSpeeds speedsFromPowers = SwerveMath.powersToSpeeds(powers, constants);
		driveByState(speedsFromPowers, swerveState);
	}

	protected void driveByState(ChassisSpeeds speeds, SwerveState swerveState) {
		this.currentState = swerveState;

		speeds = stateHandler.applyAimAssistOnChassisSpeeds(speeds, swerveState);
		speeds = handleHeadingControl(speeds, swerveState);
		if (SwerveMath.isStill(speeds, SwerveConstants.DEADBANDS)) {
			modules.stop();
			return;
		}

		speeds = SwerveMath.factorSpeeds(speeds, swerveState.getDriveSpeed());
		speeds = SwerveMath.applyDeadband(speeds, SwerveConstants.DEADBANDS);
		speeds = getDriveModeRelativeSpeeds(speeds, swerveState);
		speeds = SwerveMath.discretize(speeds);

		applySpeeds(speeds, swerveState);
	}

	private ChassisSpeeds handleHeadingControl(ChassisSpeeds speeds, SwerveState swerveState) {
		if (swerveState.getHeadingControl() == HeadingControl.NONE) {
			return speeds;
		}

		if (Math.abs(speeds.omegaRadiansPerSecond) > SwerveConstants.DEADBANDS.getRotation().getRadians()) {
			headingStabilizer.unlockTarget();
			return speeds;
		}

		headingStabilizer.setTargetHeading(headingSupplier.get());
		headingStabilizer.lockTarget();
		return new ChassisSpeeds(
			speeds.vxMetersPerSecond,
			speeds.vyMetersPerSecond,
			headingStabilizer.calculatePIDOutput(headingSupplier.get()).getRadians()
		);
	}

	private void applySpeeds(ChassisSpeeds speeds, SwerveState swerveState) {
		SwerveModuleState[] swerveModuleStates = kinematics
			.toSwerveModuleStates(speeds, stateHandler.getRotationAxis(swerveState.getRotateAxis()));
		setTargetModuleStates(swerveModuleStates, swerveState.getLoopMode().isClosedLoop());
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

	public void applyCalibrationBindings(SmartJoystick joystick, Supplier<Pose2d> robotPoseSupplier) {
		// Calibrate steer ks with phoenix tuner x
		// Calibrate steer pid with phoenix tuner x

		// Let it rotate some rotations then output will be in log under Calibrations/.
		joystick.POV_DOWN.whileTrue(getCommandsBuilder().wheelRadiusCalibration());

		// ROBOT RELATIVE DRIVE - FOR GYRO TEST
		joystick.POV_UP
			.whileTrue(commandsBuilder.driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withDriveRelative(DriveRelative.ROBOT_RELATIVE)));

		// Test the swerve returns real velocities (measure distance and time in real life and compare to swerve velocity logs).
		// REMEMBER after drive calibrations use these for pid testing - Remove OPEN LOOP for that
		joystick.POV_LEFT.whileTrue(
			getCommandsBuilder().driveByState(() -> new ChassisPowers(0.2, 0, 0), SwerveState.DEFAULT_DRIVE.withLoopMode(LoopMode.OPEN))
		);
		joystick.POV_RIGHT.whileTrue(
			getCommandsBuilder().driveByState(() -> new ChassisPowers(0.5, 0, 0), SwerveState.DEFAULT_DRIVE.withLoopMode(LoopMode.OPEN))
		);

		// Apply 12 volts on x-axis. Use it for max velocity calibrations.
		// See what velocity the swerve log after it stops accelerating and use it as max.
		joystick.START.whileTrue(commandsBuilder.maxVelocityAccelerationCalibration(VelocityType.TRANSLATIONAL));

		// Apply 12 volts on rotation-axis.
		// Use it for max velocity calibrations. See what velocity the swerve log after it stops accelerating and use it as max.
		joystick.BACK.whileTrue(commandsBuilder.maxVelocityAccelerationCalibration(VelocityType.ROTATIONAL));


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
						.pidToPose(robotPoseSupplier, robotPoseSupplier.get().plus(new Transform2d(2, 0, new Rotation2d()))),
					Set.of(this)
				)
			);
		joystick.getAxisAsButton(Axis.RIGHT_TRIGGER)
			.onTrue(
				new DeferredCommand(
					() -> getCommandsBuilder()
						.pidToPose(robotPoseSupplier, robotPoseSupplier.get().plus(new Transform2d(-2, 0, new Rotation2d()))),
					Set.of(this)
				)
			);
	}

}
