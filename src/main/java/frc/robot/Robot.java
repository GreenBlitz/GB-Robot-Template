// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.autonomous.AutonomousBuilder;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.poseestimation.PoseEstimator;
import frc.robot.structures.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.swerveconstants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.swervestatehelpers.SwerveStateHelper;
import frc.utils.auto.AutonomousChooser;
import frc.utils.auto.PathPlannerUtils;

import java.util.Optional;


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
		IGyro gyro = GyroFactory.createGyro(SwerveType.SWERVE);
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(SwerveType.SWERVE),
			ModulesFactory.create(SwerveType.SWERVE),
			gyro,
			GyroFactory.createSignals(SwerveType.SWERVE, gyro)
		);

		this.poseEstimator = new PoseEstimator(swerve::setHeading, swerve.getKinematics());

		swerve.setHeadingSupplier(() -> poseEstimator.getCurrentPose().getRotation());
		swerve.setStateHelper(new SwerveStateHelper(() -> Optional.of(poseEstimator.getCurrentPose()), Optional::empty, swerve));

		this.superStructure = new Superstructure(swerve, poseEstimator);

		buildPathPlannerForAuto();
		configureBindings();
	}

	private void buildPathPlannerForAuto() {
		swerve.configPathPlanner(poseEstimator::getCurrentPose, poseEstimator::resetPose, PathPlannerUtils.SYNCOPA_ROBOT_CONFIG);
		autonomousChooser = new AutonomousChooser(AUTONOMOUS_CHOOSER_NAME, AutonomousBuilder.getAllAutos(this));
	}

	private void configureBindings() {
		JoysticksBindings.configureBindings(this);
	}


	public PathPlannerAuto getAutonomous() {
		return autonomousChooser.getChosenValue();
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
