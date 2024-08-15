// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.poseestimation.PoseEstimator;
import frc.robot.strcutures.SuperStructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveName;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.swerveconstants.SwerveConstantsFactory;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType(RobotType.REAL);

	public static final Swerve swerve = new Swerve(
		SwerveConstantsFactory.create(SwerveName.SWERVE),
		ModulesFactory.create(SwerveName.SWERVE),
		GyroFactory.create(SwerveName.SWERVE)
	);
	public static final PoseEstimator poseEstimator = new PoseEstimator(swerve::setHeading);
	static {
		swerve.setCurrentAngleSupplier(() -> poseEstimator.getCurrentPose().getRotation());
	}

	public static final SuperStructure superStructure = new SuperStructure(swerve, poseEstimator);

	public Robot() {
		buildPathPlannerForAuto();
		configureBindings();
	}

	private void buildPathPlannerForAuto() {
		// Register commands...
		swerve.configPathPlanner(poseEstimator::getCurrentPose, poseEstimator::resetPose);
	}

	private void configureBindings() {
		JoysticksBindings.configureBindings(this);
	}


	public Command getAutonomousCommand() {
		return new InstantCommand();
	}

	public SuperStructure getSuperStructure() {
		return superStructure;
	}

	public Swerve getSwerve() {
		return swerve;
	}

	public PoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

}
