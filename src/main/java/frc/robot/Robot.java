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
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.led.LEDState;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.poseestimator.helpers.robotheadingestimator.RobotHeadingEstimatorConstants;
import frc.robot.subsystems.algaeIntake.pivot.Factory.PivotFactory;
import frc.robot.subsystems.algaeIntake.pivot.Pivot;
import frc.robot.subsystems.algaeIntake.rollers.Factory.RollersFactory;
import frc.robot.subsystems.algaeIntake.rollers.Rollers;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.subsystems.arm.factory.ArmFactory;
import frc.robot.subsystems.climb.lifter.Lifter;
import frc.robot.subsystems.climb.lifter.factory.LifterFactory;
import frc.robot.subsystems.climb.solenoid.factory.SolenoidFactory;
import frc.robot.subsystems.elevator.factory.ElevatorFactory;
import frc.robot.subsystems.endeffector.factory.EndEffectorFactory;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.modules.drive.KrakenX60DriveBuilder;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.helpers.robotheadingestimator.RobotHeadingEstimator;
import frc.robot.statemachine.RobotCommander;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.climb.solenoid.Solenoid;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.VisionFilters;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.objectdetection.LimeLightObjectDetector;
import frc.utils.TimedValue;
import frc.utils.auto.AutonomousChooser;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotManager;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.poseestimator.helpers.robotheadingestimator.RobotHeadingEstimator;
import frc.robot.poseestimator.helpers.robotheadingestimator.RobotHeadingEstimatorConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.VisionFilters;
import frc.robot.vision.multivisionsources.MultiAprilTagVisionSources;
import frc.utils.TimedValue;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.auto.PathPlannerUtil;
import frc.robot.vision.multivisionsources.MultiAprilTagVisionSources;
import frc.utils.battery.BatteryUtil;
import frc.utils.brakestate.BrakeStateManager;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;
import frc.utils.time.TimeUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final LimeLightObjectDetector objectDetector;

	private final Elevator elevator;
	private final Arm arm;
	private final EndEffector endEffector;
	private final Solenoid solenoid;
	private final Lifter lifter;
	private final Pivot pivot;
	private final Rollers rollers;

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

	private final Swerve swerve;
	private final IPoseEstimator poseEstimator;
	private final MultiAprilTagVisionSources visionSources;
	private final RobotHeadingEstimator headingEstimator;

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

		this.visionSources = new MultiAprilTagVisionSources(
			VisionConstants.MULTI_VISION_SOURCES_LOGPATH,
			headingEstimator::getEstimatedHeading,
			true,
			VisionConstants.VISION_SOURCES
		);

		visionSources.applyFunctionOnAllFilters(
			filter -> filter.and(
				data -> VisionFilters
					.isYawAtAngleForMegaTag2(
						() -> headingEstimator.getEstimatedHeadingAtTimestamp(data.getTimestamp()),
						VisionConstants.YAW_FILTER_TOLERANCE
					)
					.and(VisionFilters.isYawAngleNotZeroForMegaTag2())
					.apply(data)
			)
		);

		objectDetector = VisionConstants.LIMELIGHT_OBJECT;

		swerve.setHeadingSupplier(
			ROBOT_TYPE.isSimulation() ? () -> poseEstimator.getEstimatedPose().getRotation() : headingEstimator::getEstimatedHeading
		);

		swerve.getStateHandler().setRobotPoseSupplier(poseEstimator::getEstimatedPose);
		swerve.getStateHandler().setBranchSupplier(() -> Optional.of(ScoringHelpers.getTargetBranch()));
		swerve.getStateHandler().setReefSideSupplier(() -> Optional.of(ScoringHelpers.getTargetReefSide()));
		swerve.getStateHandler().setCoralStationSupplier(() -> Optional.of(ScoringHelpers.getTargetCoralStation(this)));
		swerve.getStateHandler().setCoralStationSlotSupplier(() -> Optional.of(ScoringHelpers.getTargetCoralStationSlot(this)));
		swerve.getStateHandler().setCageSupplier(() -> Optional.of(ScoringHelpers.getTargetCage(this)));
		swerve.getStateHandler().setClosestAlgaeSupplier(() -> objectDetector.getFilteredClosestObjectData());

		this.elevator = ElevatorFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Elevator");
		BrakeStateManager.add(() -> elevator.setBrake(true), () -> elevator.setBrake(false));

		this.arm = ArmFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Arm");
		BrakeStateManager.add(() -> arm.setBrake(true), () -> arm.setBrake(false));

		this.endEffector = EndEffectorFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/EndEffector");
		BrakeStateManager.add(() -> endEffector.setBrake(true), () -> endEffector.setBrake(false));

		this.solenoid = SolenoidFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Solenoid");

		this.lifter = LifterFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Lifter");
		BrakeStateManager.add(() -> lifter.setBrake(true), () -> lifter.setBrake(false));

		this.pivot = PivotFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/AlgaeIntake/Pivot");
		BrakeStateManager.add(() -> pivot.setBrake(true), () -> pivot.setBrake(false));

		this.rollers = RollersFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/AlgaeIntake/Rollers");
		BrakeStateManager.add(() -> rollers.setBrake(true), () -> rollers.setBrake(false));

		this.simulationManager = new SimulationManager("SimulationManager", this);
		this.robotCommander = new RobotCommander("StateMachine/RobotCommander", this);

		configureAuto();
	}

	private void configureAuto() {
		Supplier<Command> scoringCommand = () -> new WaitUntilCommand(robotCommander::isReadyToScore).andThen(
			robotCommander.getSuperstructure()
				.scoreWithoutRelease()
				.until(robotCommander.getSuperstructure()::isReadyToScore)
				.andThen(robotCommander.getSuperstructure().scoreWithRelease())
				.deadlineFor(getRobotCommander().getLedStateHandler().setState(LEDState.IN_POSITION_TO_SCORE))
				.asProxy()
		);
		Supplier<Command> intakingCommand = () -> robotCommander.getSuperstructure()
			.softCloseL4()
			.andThen(robotCommander.getSuperstructure().intake().withTimeout(AutonomousConstants.INTAKING_TIMEOUT_SECONDS))
			.asProxy();
		Supplier<Command> algaeRemoveCommand = () -> robotCommander.getSuperstructure()
			.softCloseNetToAlgaeRemove()
			.andThen(robotCommander.getSuperstructure().algaeRemove().withTimeout(AutonomousConstants.ALGAE_REMOVE_TIMEOUT_SECONDS))
			.asProxy();
		Supplier<Command> netCommand = () -> new WaitUntilCommand(robotCommander::isReadyForNetForAuto)
			.andThen(robotCommander.getSuperstructure().netWithRelease().asProxy());

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
		new EventTrigger("PRE_NET").onTrue(robotCommander.getSuperstructure().preNet());
		new EventTrigger("HOLD_ALGAE").onTrue(robotCommander.getSuperstructure().holdAlgae());
		new EventTrigger("STOP_ROLLERS").onTrue(robotCommander.getSuperstructure().algaeRemoveWithKeepRollers());
		new EventTrigger("TRANSFER_ALGAE").onTrue(robotCommander.getSuperstructure().transferAlgaeFromIntakeToEndEffector());

		this.preBuiltAutosChooser = new AutonomousChooser(
			"PreBuiltAutos",
			AutosBuilder.getAllPreBuiltAutos(
				this,
				objectDetector::getClosestObjectData,
				intakingCommand,
				scoringCommand,
				algaeRemoveCommand,
				netCommand,
				AutonomousConstants.TARGET_POSE_TOLERANCES
			)
		);
//		this.firstObjectScoringLocationChooser = new AutonomousChooser("ScoreFirst", AutosBuilder.getAllAutoScoringAutos(this));
//		this.secondObjectIntakingLocationChooser = new AutonomousChooser(
//			"IntakeSecond",
//			AutosBuilder.getAllIntakingAutos(
//				swerve,
//				poseEstimator::getEstimatedPose,
//				AutonomousConstants.getRealTimeConstraintsForAuto(swerve),
//				intakingCommand
//			)
//		);
//		this.secondObjectScoringLocationChooser = new AutonomousChooser(
//			"ScoreSecond",
//			AutosBuilder.getAllScoringAutos(
//				swerve,
//				poseEstimator::getEstimatedPose,
//				AutonomousConstants.getRealTimeConstraintsForAuto(swerve),
//				scoringCommand
//			)
//		);
//		this.thirdObjectIntakingLocationChooser = new AutonomousChooser(
//			"IntakeThird",
//			AutosBuilder.getAllIntakingAutos(
//				swerve,
//				poseEstimator::getEstimatedPose,
//				AutonomousConstants.getRealTimeConstraintsForAuto(swerve),
//				intakingCommand
//			)
//		);
//		this.thirdObjectScoringLocationChooser = new AutonomousChooser(
//			"ScoreThird",
//			AutosBuilder.getAllScoringAutos(
//				swerve,
//				poseEstimator::getEstimatedPose,
//				AutonomousConstants.getRealTimeConstraintsForAuto(swerve),
//				scoringCommand
//			)
//		);
//		this.fourthObjectIntakingLocationChooser = new AutonomousChooser(
//			"IntakeFourth",
//			AutosBuilder.getAllIntakingAutos(
//				swerve,
//				poseEstimator::getEstimatedPose,
//				AutonomousConstants.getRealTimeConstraintsForAuto(swerve),
//				intakingCommand
//			)
//		);
//		this.fourthObjectScoringLocationChooser = new AutonomousChooser(
//			"ScoreFourth",
//			AutosBuilder.getAllScoringAutos(
//				swerve,
//				poseEstimator::getEstimatedPose,
//				AutonomousConstants.getRealTimeConstraintsForAuto(swerve),
//				scoringCommand
//			)
//		);
	}

	public void periodic() {
		double startingTime = TimeUtil.getCurrentTimeSeconds();

		BusChain.refreshAll();

		swerve.update();
		arm.setReversedSoftLimit(robotCommander.getSuperstructure().getArmReversedSoftLimitByElevator());

		double poseTime = TimeUtil.getCurrentTimeSeconds();
		poseEstimator.updateOdometry(swerve.getLatestOdometryData());
		headingEstimator.updateGyroAngle(new TimedValue<>(swerve.getGyroAbsoluteYaw(), TimeUtil.getCurrentTimeSeconds()));
		for (TimedValue<Rotation2d> headingData : visionSources.getFilteredRobotHeading()) {
			headingEstimator.updateVisionIfGyroOffsetIsNotCalibrated(
				headingData,
				RobotHeadingEstimatorConstants.DEFAULT_VISION_STANDARD_DEVIATION,
				RobotHeadingEstimatorConstants.MAXIMUM_STANDARD_DEVIATION_TOLERANCE
			);
		}
		List<AprilTagVisionData> visionData = visionSources.getFilteredVisionData();
		poseEstimator.updateVision(visionData);
//		multiAprilTagVisionSources.log();
		headingEstimator.log();
		Logger.recordOutput("TimeTest/Pose", TimeUtil.getCurrentTimeSeconds() - poseTime);

		objectDetector.update();

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		simulationManager.logPoses();
		ScoringHelpers.log("Scoring");
//		ButtonDriverHelper.log("Scoring/ButtonDriverDisplay");

		double startingSchedularTime = TimeUtil.getCurrentTimeSeconds();
		CommandScheduler.getInstance().run(); // Should be last
		Logger.recordOutput("TimeTest/CommandSchedular", TimeUtil.getCurrentTimeSeconds() - startingSchedularTime);

		Logger.recordOutput("TimeTest/RobotPeriodic", TimeUtil.getCurrentTimeSeconds() - startingTime);
	}

	public PathPlannerAutoWrapper getAuto() {
		if (preBuiltAutosChooser.isDefaultOptionChosen()) {
//			if (firstObjectScoringLocationChooser.isDefaultOptionChosen()) {
			return AutosBuilder.createDefaultAuto(this);
//			}
//			return getMultiChoosersAuto();
		}
		return preBuiltAutosChooser.getChosenValue();
	}

//	private PathPlannerAutoWrapper getMultiChoosersAuto() {
//		return new PathPlannerAutoWrapper(
//			new SequentialCommandGroup(
//				firstObjectScoringLocationChooser.getChosenValue(),
//				new SequentialCommandGroup(
//					secondObjectIntakingLocationChooser.getChosenValue(),
//					secondObjectScoringLocationChooser.getChosenValue(),
//					thirdObjectIntakingLocationChooser.getChosenValue(),
//					thirdObjectScoringLocationChooser.getChosenValue(),
//					fourthObjectIntakingLocationChooser.getChosenValue(),
//					fourthObjectScoringLocationChooser.getChosenValue()
//				).asProxy()
//			),
//			firstObjectScoringLocationChooser.getChosenValue().getStartingPose(),
//			"Multi Choosers Auto"
//		);
//	}

	public IPoseEstimator getPoseEstimator() {
		return poseEstimator;
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

	public Pivot getPivot() {
		return pivot;
	}

	public Rollers getRollers() {
		return rollers;
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

	public Swerve getSwerve() {
		return swerve;
	}

}
