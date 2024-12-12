// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autonomous.autos.M231Auto;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.poseestimation.PoseEstimator;
import frc.robot.structures.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.swerveconstants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.swervestatehelpers.SwerveStateHelper;
import frc.utils.auto.PathPlannerUtils;

import java.util.Optional;


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

	public Robot() {
		IGyro gyro = GyroFactory.createGyro(SwerveType.SWERVE);
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(SwerveType.SWERVE),
			ModulesFactory.create(SwerveType.SWERVE),
			gyro,
			GyroFactory.createSignals(SwerveType.SWERVE, gyro)
		);

		this.poseEstimator = new PoseEstimator(swerve::setHeading, swerve.getConstants().kinematics());

		swerve.setHeadingSupplier(() -> poseEstimator.getCurrentPose().getRotation());
		swerve.setStateHelper(new SwerveStateHelper(() -> Optional.of(poseEstimator.getCurrentPose()), Optional::empty, swerve));

		this.superStructure = new Superstructure(swerve, poseEstimator);

		buildPathPlannerForAuto();
		configureBindings();
	}

	private void buildPathPlannerForAuto() {
		// Register commands...
		NamedCommands.registerCommand("wait", new WaitCommand(3));
		NamedCommands.registerCommand("they don't love you", new WaitCommand(1));
		NamedCommands.registerCommand("like I love you", new WaitCommand(1.5));
		swerve.configPathPlanner(poseEstimator::getCurrentPose, poseEstimator::resetPose, PathPlannerUtils.SYNCOPA_ROBOT_CONFIG);
	}

	private void configureBindings() {
		JoysticksBindings.configureBindings(this);
	}


	public Command getAutonomousCommand() {
		PathPlannerAuto auto = new M231Auto(this);
		return AutoBuilder.resetOdom(auto.getStartingPose()).andThen(auto);
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
