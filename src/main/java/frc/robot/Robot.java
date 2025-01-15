// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

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
import frc.robot.subsystems.swerve.factories.swerveconstants.SwerveConstantsFactory;
import frc.utils.auto.AutonomousChooser;
import frc.utils.auto.GBAuto;
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

	private static final String AUTONOMOUS_CHOOSER_NAME = "autonomousChooser";

	private final Swerve swerve;
	private final PoseEstimator poseEstimator;
	private final Superstructure superStructure;

	private AutonomousChooser autonomousChooser;

	public Robot() {
		BatteryUtils.scheduleLimiter();

		IGyro gyro = GyroFactory.createGyro(RobotConstants.SUBSYSTEM_LOG_PREFIX + "Swerve/");
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(RobotConstants.SUBSYSTEM_LOG_PREFIX + "Swerve/"),
			ModulesFactory.create(RobotConstants.SUBSYSTEM_LOG_PREFIX + "Swerve/"),
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
		Supplier<Command> intakeCommand = () -> superStructure.setState(RobotState.INTAKE);
		Supplier<Command> shootingCommand = () -> superStructure.setState(RobotState.SPEAKER);

		swerve.configPathPlanner(
			poseEstimator::getCurrentPose,
			poseEstimator::resetPose,
			PathPlannerUtils.getGuiRobotConfig().orElse(AutonomousConstants.SYNCOPA_ROBOT_CONFIG)
		);
		autonomousChooser = new AutonomousChooser(AUTONOMOUS_CHOOSER_NAME, AutosBuilder.getAllAutos(this, intakeCommand, shootingCommand));
	}


	public void periodic() {
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		superStructure.periodic();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public GBAuto getAuto() {
		return autonomousChooser.getChosenValue().withResetPose(poseEstimator::resetPose);
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
