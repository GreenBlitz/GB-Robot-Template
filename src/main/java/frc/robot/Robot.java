// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.RobotManager;
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.autonomous.AutosBuilder;
import frc.robot.poseestimator.helpers.RobotHeadingEstimator.RobotHeadingEstimatorConstants;
import frc.robot.scoringhelpers.ButtonDriverHelper;
import frc.robot.vision.VisionConstants;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.poseestimator.helpers.RobotHeadingEstimator.RobotHeadingEstimator;
import frc.robot.statemachine.RobotCommander;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.factory.ArmFactory;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.factory.ElevatorFactory;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.factory.EndEffectorFactory;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.utils.auto.AutonomousChooser;
import frc.utils.auto.PathPlannerUtil;
import frc.robot.vision.VisionFilters;
import frc.robot.vision.data.VisionData;
import frc.robot.vision.multivisionsources.MultiAprilTagVisionSources;
import frc.utils.Filter;
import frc.utils.TimedValue;
import frc.utils.brakestate.BrakeStateManager;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;
import frc.utils.time.TimeUtil;

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

	private final SimulationManager simulationManager;
	private final RobotCommander robotCommander;

	private AutonomousChooser startingPointAndWhereToScoreFirstObjectChooser;
	private AutonomousChooser whereToIntakeSecondObjectChooser;
	private AutonomousChooser whereToScoreSecondObjectChooser;
	private AutonomousChooser whereToIntakeThirdObjectChooser;
	private AutonomousChooser whereToScoreThirdObjectChooser;
	private AutonomousChooser whereToIntakeFourthObjectChooser;
	private AutonomousChooser whereToScoreFourthObjectChooser;

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
				new Filter<>(
					data -> VisionFilters.isYawAtAngle(headingEstimator::getEstimatedHeading, VisionConstants.ANGLE_FILTERS_TOLERANCES)
						.apply((VisionData) data)
				)
			)
		);

		swerve.setHeadingSupplier(
			() -> ROBOT_TYPE.isSimulation() ? poseEstimator.getEstimatedPose().getRotation() : headingEstimator.getEstimatedHeading()
		);
		swerve.getStateHandler().setRobotPoseSupplier(poseEstimator::getEstimatedPose);
		swerve.getStateHandler().setBranchSupplier(() -> Optional.of(ScoringHelpers.getTargetBranch()));
		swerve.getStateHandler().setReefSideSupplier(() -> Optional.of(ScoringHelpers.getTargetReefSide()));
		swerve.getStateHandler().setCoralStationSupplier(() -> Optional.of(ScoringHelpers.getTargetCoralStation(this)));

		this.elevator = ElevatorFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Elevator");
		BrakeStateManager.add(() -> elevator.setBrake(true), () -> elevator.setBrake(false));

		this.arm = ArmFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Arm");
		BrakeStateManager.add(() -> arm.setBrake(true), () -> arm.setBrake(false));

		this.endEffector = EndEffectorFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/EndEffector");

		this.simulationManager = new SimulationManager("SimulationManager", this);
		this.robotCommander = new RobotCommander("StateMachine/RobotCommander", this);

		configureAuto();
	}

	private void configureAuto() {
		Supplier<Command> scoringCommand = () -> new InstantCommand(() -> ScoringHelpers.targetScoreLevel = ScoreLevel.L4)
			.andThen(
				robotCommander.getSuperstructure()
					.scoreWithRelease()
					.andThen(robotCommander.getSuperstructure().preScore().until(() -> robotCommander.getSuperstructure().isPreScoreReady()))
			)
			.asProxy();
		Supplier<Command> intakingCommand = () -> robotCommander.getSuperstructure().intake().asProxy();

		swerve.configPathPlanner(
			poseEstimator::getEstimatedPose,
			poseEstimator::resetPose,
			PathPlannerUtil.getGuiRobotConfig().orElse(AutonomousConstants.ROBOT_CONFIG)
		);

		new EventTrigger("PRE_SCORE").onTrue(
			new InstantCommand(() -> ScoringHelpers.targetScoreLevel = ScoreLevel.L4).andThen(robotCommander.getSuperstructure().preScore())
		);
		new EventTrigger("INTAKE").onTrue(robotCommander.getSuperstructure().intake());
		new EventTrigger("ARM_PRE_SCORE").onTrue(robotCommander.getSuperstructure().armPreScore());

		this.startingPointAndWhereToScoreFirstObjectChooser = new AutonomousChooser(
			"StartingPointAndScoreFirst",
			AutosBuilder.getAllStartingAndScoringFirstObjectAutos(this, scoringCommand, AutonomousConstants.TARGET_POSE_TOLERANCES)
		);
		this.whereToIntakeSecondObjectChooser = new AutonomousChooser(
			"IntakeSecond",
			AutosBuilder.getAllIntakingAutos(this, intakingCommand, AutonomousConstants.TARGET_POSE_TOLERANCES)
		);
		this.whereToScoreSecondObjectChooser = new AutonomousChooser(
			"ScoreSecond",
			AutosBuilder.getAllScoringAutos(this, scoringCommand, AutonomousConstants.TARGET_POSE_TOLERANCES)
		);
		this.whereToIntakeThirdObjectChooser = new AutonomousChooser(
			"IntakeThird",
			AutosBuilder.getAllIntakingAutos(this, intakingCommand, AutonomousConstants.TARGET_POSE_TOLERANCES)
		);
		this.whereToScoreThirdObjectChooser = new AutonomousChooser(
			"ScoreThird",
			AutosBuilder.getAllScoringAutos(this, scoringCommand, AutonomousConstants.TARGET_POSE_TOLERANCES)
		);
		this.whereToIntakeFourthObjectChooser = new AutonomousChooser(
			"IntakeFourth",
			AutosBuilder.getAllIntakingAutos(this, intakingCommand, AutonomousConstants.TARGET_POSE_TOLERANCES)
		);
		this.whereToScoreFourthObjectChooser = new AutonomousChooser(
			"ScoreFourth",
			AutosBuilder.getAllScoringAutos(this, scoringCommand, AutonomousConstants.TARGET_POSE_TOLERANCES)
		);
	}

	public void periodic() {
		swerve.update();
		arm.setReversedSoftLimit(robotCommander.getSuperstructure().getArmReversedSoftLimitByElevator());

		headingEstimator.updateGyroAngle(new TimedValue<>(swerve.getGyroAbsoluteYaw(), TimeUtil.getCurrentTimeSeconds()));
		for (TimedValue<Rotation2d> headingData : multiAprilTagVisionSources.getRawRobotHeadings()) {
			headingEstimator.updateVisionIfNotCalibrated(
				headingData,
				RobotHeadingEstimatorConstants.DEFAULT_VISION_STANDARD_DEVIATION,
				RobotHeadingEstimatorConstants.MAXIMUM_STANDARD_DEVIATION_TOLERANCE
			);
		}
		poseEstimator.updateOdometry(swerve.getAllOdometryData());
		poseEstimator.updateVision(multiAprilTagVisionSources.getFilteredVisionData());
		multiAprilTagVisionSources.log();
		headingEstimator.log();

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		simulationManager.logPoses();
		ScoringHelpers.log("Scoring");
		ButtonDriverHelper.log("Scoring/ButtonDriverDisplay");

		CommandScheduler.getInstance().run(); // Should be last
	}

	public PathPlannerAutoWrapper getAuto() {
		return PathPlannerAutoWrapper
			.chainAutos(
				startingPointAndWhereToScoreFirstObjectChooser.getChosenValue(),
				whereToIntakeSecondObjectChooser.getChosenValue(),
				whereToScoreSecondObjectChooser.getChosenValue(),
				whereToIntakeThirdObjectChooser.getChosenValue(),
				whereToScoreThirdObjectChooser.getChosenValue(),
				whereToIntakeFourthObjectChooser.getChosenValue(),
				whereToScoreFourthObjectChooser.getChosenValue()
			)
			.withResetPose(poseEstimator::resetPose);
	}

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

	public RobotCommander getRobotCommander() {
		return robotCommander;
	}

}
