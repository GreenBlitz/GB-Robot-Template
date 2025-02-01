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
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.poseestimation.PoseEstimator;
import frc.robot.structures.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.utils.DriverStationUtils;
import frc.utils.auto.AutonomousChooser;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.battery.BatteryUtil;

import java.util.function.Supplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final Swerve swerve;
	private final PoseEstimator poseEstimator;
	private final Superstructure superStructure;

	private AutonomousChooser testAutosChooser;
	private AutonomousChooser startingPointAndWhereToScoreFirstObjectChooser;
	private AutonomousChooser whereToIntakeSecondObjectChooser;
	private AutonomousChooser whereToScoreSecondObjectChooser;

	public Robot() {
		BatteryUtil.scheduleLimiter();

		IGyro gyro = GyroFactory.createGyro(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve");
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			ModulesFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			gyro,
			GyroFactory.createSignals(gyro)
		);

		this.poseEstimator = new PoseEstimator(swerve::setHeading, swerve.getKinematics());

		swerve.setHeadingSupplier(() -> poseEstimator.getCurrentPose().getRotation());
		swerve.getStateHandler().setRobotPoseSupplier(poseEstimator::getCurrentPose);

		this.superStructure = new Superstructure(swerve, poseEstimator);

		configureAuto();
	}


	private void configureAuto() {
		Supplier<Command> scoreL4Command = () -> superStructure.setState(RobotState.SCORE_L4);
		Supplier<Command> intakeCommand = () -> superStructure.setState(RobotState.INTAKE);

		Command preIntakeCommand = superStructure.setState(RobotState.PRE_INTAKE);

		swerve.configPathPlanner(
			poseEstimator::getCurrentPose,
			poseEstimator::resetPose,
			PathPlannerUtil.getGuiRobotConfig().orElse(AutonomousConstants.SYNCOPA_ROBOT_CONFIG)
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
		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		superStructure.periodic();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public PathPlannerAutoWrapper getAuto() {
		boolean isAutoChosen = !startingPointAndWhereToScoreFirstObjectChooser.isDefaultOptionChosen();
		if (isAutoChosen) {
			return PathPlannerAutoWrapper
				.chainAutos(
					startingPointAndWhereToScoreFirstObjectChooser.getChosenValue(),
					whereToIntakeSecondObjectChooser.getChosenValue(),
					whereToScoreSecondObjectChooser.getChosenValue()
				)
				.withResetPose(getPoseEstimator()::resetPose);
		}
		if (!DriverStationUtils.isMatch()) {
			return testAutosChooser.getChosenValue();
		}
		return new PathPlannerAutoWrapper();
	}

	public Superstructure getSuperStructure() {
		return superStructure;
	}

	public Swerve getSwerve() {
		return swerve;
	}

	public PoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

}
