// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.RobotManager;
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.autonomous.AutosBuilder;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
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
import frc.utils.brakestate.BrakeStateManager;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;

import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final WPILibPoseEstimatorWrapper poseEstimator;

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

		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
		swerve.getStateHandler().setRobotPoseSupplier(poseEstimator::getEstimatedPose);

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
		Supplier<Command> scoringCommand = () -> robotCommander.getSuperstructure().scoreL4().andThen(robotCommander.getSuperstructure().preL4().until(() -> robotCommander.getSuperstructure().isPreScoreReady(ScoreLevel.L4)));

		swerve.configPathPlanner(
			poseEstimator::getEstimatedPose,
			poseEstimator::resetPose,
			PathPlannerUtil.getGuiRobotConfig().orElse(AutonomousConstants.ROBOT_CONFIG)
		);

		new EventTrigger("PRE_SCORE").onTrue(robotCommander.getSuperstructure().preL4());
		new EventTrigger("INTAKE").onTrue(robotCommander.getSuperstructure().intake());
		new EventTrigger("IDLE").onTrue(robotCommander.getSuperstructure().idle());

		this.startingPointAndWhereToScoreFirstObjectChooser = new AutonomousChooser(
			"StartingPointAndScoreFirst",
			AutosBuilder.getAllStartingAndScoringFirstObjectAutos(this, scoringCommand, AutonomousConstants.TARGET_POSE_TOLERANCES)
		);
		this.whereToIntakeSecondObjectChooser = new AutonomousChooser(
			"IntakeSecond",
			AutosBuilder.getAllIntakingAutos(this, AutonomousConstants.TARGET_POSE_TOLERANCES)
		);
		this.whereToScoreSecondObjectChooser = new AutonomousChooser(
			"ScoreSecond",
			AutosBuilder.getAllScoringAutos(this, scoringCommand, AutonomousConstants.TARGET_POSE_TOLERANCES)
		);
		this.whereToIntakeThirdObjectChooser = new AutonomousChooser(
			"IntakeThird",
			AutosBuilder.getAllIntakingAutos(this, AutonomousConstants.TARGET_POSE_TOLERANCES)
		);
		this.whereToScoreThirdObjectChooser = new AutonomousChooser(
			"ScoreThird",
			AutosBuilder.getAllScoringAutos(this, scoringCommand, AutonomousConstants.TARGET_POSE_TOLERANCES)
		);
		this.whereToIntakeFourthObjectChooser = new AutonomousChooser(
			"IntakeFourth",
			AutosBuilder.getAllIntakingAutos(this, AutonomousConstants.TARGET_POSE_TOLERANCES)
		);
		this.whereToScoreFourthObjectChooser = new AutonomousChooser(
			"ScoreFourth",
			AutosBuilder.getAllScoringAutos(this, scoringCommand, AutonomousConstants.TARGET_POSE_TOLERANCES)
		);
	}

	public void periodic() {
		swerve.update();
		poseEstimator.updateOdometry(swerve.getAllOdometryObservations());
		arm.setReversedSoftLimit(robotCommander.getSuperstructure().getArmReversedSoftLimitByElevator());
		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		simulationManager.logPoses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public PathPlannerAutoWrapper getAuto() {
		return PathPlannerAutoWrapper.chainAutos(
			startingPointAndWhereToScoreFirstObjectChooser.getChosenValue(),
			whereToIntakeSecondObjectChooser.getChosenValue(),
			whereToScoreSecondObjectChooser.getChosenValue(),
			whereToIntakeThirdObjectChooser.getChosenValue(),
			whereToScoreThirdObjectChooser.getChosenValue(),
			whereToIntakeFourthObjectChooser.getChosenValue(),
			whereToScoreFourthObjectChooser.getChosenValue()
		).withResetPose(poseEstimator::resetPose);
	}

	public WPILibPoseEstimatorWrapper getPoseEstimator() {
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
