// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.*;
import frc.RobotManager;
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.autonomous.AutosBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.led.LEDState;
import frc.robot.poseestimator.helpers.RobotHeadingEstimator.RobotHeadingEstimatorConstants;
import frc.robot.subsystems.climb.lifter.Lifter;
import frc.robot.subsystems.climb.lifter.factory.LifterFactory;
import frc.robot.subsystems.swerve.factories.modules.drive.KrakenX60DriveBuilder;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.robot.vision.VisionConstants;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.poseestimator.helpers.RobotHeadingEstimator.RobotHeadingEstimator;
import frc.robot.statemachine.RobotCommander;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.factory.ArmFactory;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.factory.ElevatorFactory;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.factory.EndEffectorFactory;
import frc.robot.subsystems.climb.solenoid.Solenoid;
import frc.robot.subsystems.climb.solenoid.factory.SolenoidFactory;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.vision.data.AprilTagVisionData;
import frc.utils.auto.AutonomousChooser;
import frc.utils.auto.PathPlannerUtil;
import frc.robot.vision.VisionFilters;
import frc.robot.vision.multivisionsources.MultiAprilTagVisionSources;
import frc.utils.TimedValue;
import frc.utils.brakestate.BrakeStateManager;
import frc.utils.battery.BatteryUtil;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final IPoseEstimator poseEstimator;
	private final RobotHeadingEstimator headingEstimator;
	private final MultiAprilTagVisionSources multiAprilTagVisionSources;

	private final Swerve swerve;
	private final Elevator elevator;
	private final Arm arm;
	private final EndEffector endEffector;
	private final Solenoid solenoid;
	private final Lifter lifter;

	private final SimulationManager simulationManager;
	private final RobotCommander robotCommander;

	private AutonomousChooser preBuiltAutosChooser;
	private AutonomousChooser firstObjectScoringLocationChooser;
	private AutonomousChooser secondObjectIntakingLocationChooser;
	private AutonomousChooser secondObjectScoringLocationChooser;
	private AutonomousChooser thirdObjectIntakingLocationChooser;
	private AutonomousChooser thirdObjectScoringLocationChooser;
	private AutonomousChooser fourthObjectIntakingLocationChooser;
	private AutonomousChooser fourthObjectScoringLocationChooser;

	public Robot() {
		BatteryUtil.scheduleLimiter();

		IGyro gyro = GyroFactory.createGyro(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve");
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			ModulesFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			gyro,
			GyroFactory.createSignals(gyro)
		);

		this.poseEstimator = new WPILibPoseEstimatorWrapper(
			WPILibPoseEstimatorConstants.WPILIB_POSEESTIMATOR_LOGPATH,
			swerve.getKinematics(),
			swerve.getModules().getWheelPositions(0),
			swerve.getGyroAbsoluteYaw()
		);

		this.headingEstimator = new RobotHeadingEstimator(
			RobotHeadingEstimatorConstants.DEFAULT_HEADING_ESTIMATOR_LOGPATH,
			new Rotation2d(),
			new Rotation2d(),
			RobotHeadingEstimatorConstants.DEFAULT_GYRO_STANDARD_DEVIATION
		);

		this.multiAprilTagVisionSources = new MultiAprilTagVisionSources(
			VisionConstants.MULTI_VISION_SOURCES_LOGPATH,
			headingEstimator::getEstimatedHeading,
			true,
			VisionConstants.VISION_SOURCES
		);

		multiAprilTagVisionSources.applyFunctionOnAllFilters(
			filters -> filters.and(
				data -> VisionFilters
					.isYawAtAngleForMegaTag2(
						() -> headingEstimator.getEstimatedHeadingAtTimestamp(data.getTimestamp()),
						VisionConstants.YAW_FILTER_TOLERANCE
					)
					.and(VisionFilters.isYawAngleNotZeroForMegaTag2())
					.apply(data)
			)
		);

		swerve.setHeadingSupplier(
			ROBOT_TYPE.isSimulation() ? () -> poseEstimator.getEstimatedPose().getRotation() : headingEstimator::getEstimatedHeading
		);

		swerve.getStateHandler().setRobotPoseSupplier(poseEstimator::getEstimatedPose);
		swerve.getStateHandler().setBranchSupplier(() -> Optional.of(ScoringHelpers.getTargetBranch()));
		swerve.getStateHandler().setReefSideSupplier(() -> Optional.of(ScoringHelpers.getTargetReefSide()));
		swerve.getStateHandler().setCoralStationSupplier(() -> Optional.of(ScoringHelpers.getTargetCoralStation(this)));
		swerve.getStateHandler().setCoralStationSlotSupplier(() -> Optional.of(ScoringHelpers.getTargetCoralStationSlot(this)));
		swerve.getStateHandler().setCageSupplier(() -> Optional.of(ScoringHelpers.getTargetCage(this)));

		this.elevator = ElevatorFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Elevator");
		BrakeStateManager.add(() -> elevator.setBrake(true), () -> elevator.setBrake(false));

		this.arm = ArmFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Arm");
		BrakeStateManager.add(() -> arm.setBrake(true), () -> arm.setBrake(false));

		this.endEffector = EndEffectorFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/EndEffector");
		BrakeStateManager.add(() -> endEffector.setBrake(true), () -> endEffector.setBrake(false));

		this.solenoid = SolenoidFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Solenoid");

		this.lifter = LifterFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Lifter");
		BrakeStateManager.add(() -> lifter.setBrake(true), () -> lifter.setBrake(false));

		this.simulationManager = new SimulationManager("SimulationManager", this);
		this.robotCommander = new RobotCommander("StateMachine/RobotCommander", this);

		configureAuto();
	}

	private void configureAuto() {
		Supplier<Command> scoringCommand = () -> robotCommander.getSuperstructure()
			.scoreWithRelease()
			.deadlineFor(getRobotCommander().getLedStateHandler().setState(LEDState.IN_POSITION_TO_SCORE))
			.asProxy();
		Supplier<Command> intakingCommand = () -> robotCommander.getSuperstructure()
			.softCloseL4()
			.andThen(robotCommander.getSuperstructure().intake().withTimeout(AutonomousConstants.INTAKING_TIMEOUT_SECONDS))
			.asProxy();

		swerve.configPathPlanner(
			poseEstimator::getEstimatedPose,
			poseEstimator::resetPose,
			PathPlannerUtil.getGuiRobotConfig().orElse(getRobotConfig())
		);

		new EventTrigger("PULL_OUT_ARM")
			.onTrue(robotCommander.getSuperstructure().closeClimb().andThen(robotCommander.getSuperstructure().armPreScore()));
		new EventTrigger("PRE_SCORE").onTrue(
			robotCommander.getSuperstructure()
				.preScore()
				.alongWith(getRobotCommander().getLedStateHandler().setState(LEDState.IN_POSITION_TO_OPEN_ELEVATOR))
				.until(() -> robotCommander.getSuperstructure().isPreScoreReady())
				.andThen(
					robotCommander.getSuperstructure()
						.scoreWithoutRelease()
						.alongWith(getRobotCommander().getLedStateHandler().setState(LEDState.OPENING_SUPERSTRUCTURE))
				)
		);
		new EventTrigger("ARM_PRE_SCORE").onTrue(
			robotCommander.getSuperstructure().armPreScore().alongWith(getRobotCommander().getLedStateHandler().setState(LEDState.MOVE_TO_POSE))
		);

		this.preBuiltAutosChooser = new AutonomousChooser(
			"PreBuiltAutos",
			AutosBuilder.getAllNoDelayAutos(this, intakingCommand, scoringCommand, AutonomousConstants.TARGET_POSE_TOLERANCES)
		);
//		this.firstObjectScoringLocationChooser = new AutonomousChooser("ScoreFirst", AutosBuilder.getAllAutoScoringAutos(this));
//		this.secondObjectIntakingLocationChooser = new AutonomousChooser(
//			"IntakeSecond",
//			AutosBuilder.getAllIntakingAutos(this, intakingCommand, AutonomousConstants.TARGET_POSE_TOLERANCES)
//		);
//		this.secondObjectScoringLocationChooser = new AutonomousChooser(
//			"ScoreSecond",
//			AutosBuilder.getAllScoringAutos(this, scoringCommand, AutonomousConstants.TARGET_POSE_TOLERANCES)
//		);
//		this.thirdObjectIntakingLocationChooser = new AutonomousChooser(
//			"IntakeThird",
//			AutosBuilder.getAllIntakingAutos(this, intakingCommand, AutonomousConstants.TARGET_POSE_TOLERANCES)
//		);
//		this.thirdObjectScoringLocationChooser = new AutonomousChooser(
//			"ScoreThird",
//			AutosBuilder.getAllScoringAutos(this, scoringCommand, AutonomousConstants.TARGET_POSE_TOLERANCES)
//		);
//		this.fourthObjectIntakingLocationChooser = new AutonomousChooser(
//			"IntakeFourth",
//			AutosBuilder.getAllIntakingAutos(this, intakingCommand, AutonomousConstants.TARGET_POSE_TOLERANCES)
//		);
//		this.fourthObjectScoringLocationChooser = new AutonomousChooser(
//			"ScoreFourth",
//			AutosBuilder.getAllScoringAutos(this, scoringCommand, AutonomousConstants.TARGET_POSE_TOLERANCES)
//		);
	}

	public void periodic() {
		double startingTime = TimeUtil.getCurrentTimeSeconds();

		Phoenix6SignalBuilder.refreshAll();

		swerve.update();
		arm.setReversedSoftLimit(robotCommander.getSuperstructure().getArmReversedSoftLimitByElevator());

		double poseTime = TimeUtil.getCurrentTimeSeconds();
		poseEstimator.updateOdometry(swerve.getLatestOdometryData());
		headingEstimator.updateGyroAngle(new TimedValue<>(swerve.getGyroAbsoluteYaw(), TimeUtil.getCurrentTimeSeconds()));
		for (TimedValue<Rotation2d> headingData : multiAprilTagVisionSources.getFilteredRobotHeading()) {
			headingEstimator.updateVisionIfGyroOffsetIsNotCalibrated(
				headingData,
				RobotHeadingEstimatorConstants.DEFAULT_VISION_STANDARD_DEVIATION,
				RobotHeadingEstimatorConstants.MAXIMUM_STANDARD_DEVIATION_TOLERANCE
			);
		}
		List<AprilTagVisionData> visionData = multiAprilTagVisionSources.getFilteredVisionData();
		poseEstimator.updateVision(visionData);
//		 multiAprilTagVisionSources.log();
		headingEstimator.log();
		Logger.recordOutput("TimeTest/Pose", TimeUtil.getCurrentTimeSeconds() - poseTime);

		BatteryUtil.logStatus();
//		BusChain.logChainsStatuses();
		simulationManager.logPoses();
		ScoringHelpers.log("Scoring");
//		ButtonDriverHelper.log("Scoring/ButtonDriverDisplay");

		double startingSchedularTime = TimeUtil.getCurrentTimeSeconds();
		CommandScheduler.getInstance().run(); // Should be last
		Logger.recordOutput("TimeTest/CommandSchedular", TimeUtil.getCurrentTimeSeconds() - startingSchedularTime);

		Logger.recordOutput("TimeTest/RobotPeriodic", TimeUtil.getCurrentTimeSeconds() - startingTime);
	}

	public Command getAuto() {
		if (preBuiltAutosChooser.isDefaultOptionChosen()) {
//			if (firstObjectScoringLocationChooser.isDefaultOptionChosen()) {
			return AutosBuilder.createDefaultNoDelayAuto(this);
//			}
//			return getMultiChoosersAuto();
		}
		return preBuiltAutosChooser.getChosenValue();
	}

//	private PathPlannerAutoWrapper getMultiChoosersAuto() {
//		return PathPlannerAutoWrapper.chainAutos(
//			firstObjectScoringLocationChooser.getChosenValue(),
//			PathPlannerAutoWrapper
//				.chainAutos(
//					secondObjectIntakingLocationChooser.getChosenValue(),
//					secondObjectScoringLocationChooser.getChosenValue(),
//					thirdObjectIntakingLocationChooser.getChosenValue(),
//					thirdObjectScoringLocationChooser.getChosenValue(),
//					fourthObjectIntakingLocationChooser.getChosenValue(),
//					fourthObjectScoringLocationChooser.getChosenValue()
//				)
//				.asProxyAuto()
//		);
//	}

	public IPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

	public Swerve getSwerve() {
		return swerve;
	}

	public Elevator getElevator() {
		return elevator;
	}

	public Arm getArm() {
		return arm;
	}

	public EndEffector getEndEffector() {
		return endEffector;
	}

	public Solenoid getSolenoid() {
		return solenoid;
	}

	public Lifter getLifter() {
		return lifter;
	}

	public RobotCommander getRobotCommander() {
		return robotCommander;
	}

	public RobotConfig getRobotConfig() {
		return new RobotConfig(
			RobotConstants.MASS_KILOGRAM,
			RobotConstants.MOMENT_OF_INERTIA_KILOGRAM_METERS_SQUARED,
			new ModuleConfig(
				swerve.getModules().getModule(ModuleUtil.ModulePosition.FRONT_LEFT).getModuleConstants().wheelDiameterMeters() / 2,
				swerve.getConstants().velocityAt12VoltsMetersPerSecond(),
				ModuleConstants.COEFFICIENT_OF_FRICTION,
				DCMotor.getKrakenX60Foc(ModuleConstants.NUMBER_OF_DRIVE_MOTORS),
				KrakenX60DriveBuilder.GEAR_RATIO,
				KrakenX60DriveBuilder.SLIP_CURRENT,
				ModuleConstants.NUMBER_OF_DRIVE_MOTORS
			),
			swerve.getModules().getModulePositionsFromCenterMeters()
		);
	}

}
