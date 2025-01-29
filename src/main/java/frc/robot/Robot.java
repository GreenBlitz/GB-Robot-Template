// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.events.EventTrigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotManager;
import frc.robot.autonomous.AutosBuilder;
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.autonomous.SequencesBuilder;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.structures.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.utils.DriverStationUtils;
import frc.utils.auto.AutoPath;
import frc.utils.auto.AutonomousChooser;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.auto.PathPlannerUtils;
import frc.utils.battery.BatteryUtils;

import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final Swerve swerve;
	private final IPoseEstimator poseEstimator;
	private final Superstructure superStructure;

	private AutonomousChooser testAutosChooser;
	private AutonomousChooser startingPointAndWhereToScoreFirstObjectChooser;
	private AutonomousChooser whereToIntakeSecondObjectChooser;
	private AutonomousChooser whereToScoreSecondObjectChooser;

	public Robot() {
		BatteryUtils.scheduleLimiter();

		IGyro gyro = GyroFactory.createGyro(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve");
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			ModulesFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			gyro,
			GyroFactory.createSignals(gyro)
		);

		this.poseEstimator = new WPILibPoseEstimatorWrapper(
			"poseEstimator/",
			swerve.getKinematics(),
			swerve.getAllOdometryObservations()[0].wheelPositions(),
			swerve.getGyroAbsoluteYaw()
		);

		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
		swerve.getStateHandler().setRobotPoseSupplier(poseEstimator::getEstimatedPose);

		this.superStructure = new Superstructure(swerve, poseEstimator);

		configureAuto();
	}


	private void configureAuto() {
		Supplier<Command> scoreL4Command = () -> superStructure.setState(RobotState.SCORE_L4);
		Supplier<Command> intakeCommand = () -> superStructure.setState(RobotState.INTAKE);

		Command preIntakeCommand = superStructure.setState(RobotState.PRE_INTAKE);

		swerve.configPathPlanner(
			poseEstimator::getEstimatedPose,
			poseEstimator::resetPose,
			PathPlannerUtils.getGuiRobotConfig().orElse(AutonomousConstants.SYNCOPA_ROBOT_CONFIG)
		);

		new EventTrigger("Intake").onTrue(preIntakeCommand);

		testAutosChooser = new AutonomousChooser("TestAutosChooser", AutosBuilder.getAllTestAutos());
		startingPointAndWhereToScoreFirstObjectChooser = new AutonomousChooser(
			"StartingPointAndWhereToScoreFirstObjectChooser",
			AutosBuilder.getAllStartingAndScoringFirstObjectAutos(this, scoreL4Command)
		);
		whereToIntakeSecondObjectChooser = new AutonomousChooser(
			"WhereToIntakeSecondObjectChooser",
			AutosBuilder.getAllIntakingAutos(this, intakeCommand)
		);
		whereToScoreSecondObjectChooser = new AutonomousChooser(
			"WhereToScoreSecondObjectChooser",
			AutosBuilder.getAllScoringAutos(this, scoreL4Command)
		);
	}


	public void periodic() {
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		superStructure.periodic();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public PathPlannerAutoWrapper getAuto() {
//		boolean isAutoChosen = !startingPointAndWhereToScoreFirstObjectChooser.isDefaultOptionChosen();
//		if (isAutoChosen) {
//			return PathPlannerAutoWrapper
//				.chainAutos(
//					startingPointAndWhereToScoreFirstObjectChooser.getChosenValue(),
//					whereToIntakeSecondObjectChooser.getChosenValue(),
//					whereToScoreSecondObjectChooser.getChosenValue()
//				)
//				.withResetPose(getPoseEstimator()::resetPose);
//		}
//		if (!DriverStationUtils.isMatch()) {
//			return testAutosChooser.getChosenValue();
//		}
//		return new PathPlannerAutoWrapper();
		return PathPlannerAutoWrapper.chainAutos(
				AutosBuilder.createAutoFromAutoPath(AutoPath.AUTO_LINE_1_TO_I, SequencesBuilder::followPath),
				AutosBuilder.createAutoFromAutoPath(AutoPath.I_TO_UPPER_CORAL_STATION, SequencesBuilder::followPath),
				AutosBuilder.createAutoFromAutoPath(AutoPath.UPPER_CORAL_STATION_TO_L, SequencesBuilder::followPath),
				AutosBuilder.createAutoFromAutoPath(AutoPath.L_TO_UPPER_CORAL_STATION, SequencesBuilder::followPath),
				AutosBuilder.createAutoFromAutoPath(AutoPath.UPPER_CORAL_STATION_TO_K, SequencesBuilder::followPath),
				AutosBuilder.createAutoFromAutoPath(AutoPath.K_TO_UPPER_CORAL_STATION, SequencesBuilder::followPath),
				AutosBuilder.createAutoFromAutoPath(AutoPath.UPPER_CORAL_STATION_TO_J, SequencesBuilder::followPath)
		).withResetPose(poseEstimator::resetPose);
	}

	public Superstructure getSuperStructure() {
		return superStructure;
	}

	public Swerve getSwerve() {
		return swerve;
	}

	public IPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

}
