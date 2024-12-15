// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.poseestimation.PoseEstimator;
import frc.robot.poseestimator.*;
import frc.robot.structures.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.swerveconstants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.swervestatehelpers.SwerveStateHelper;
import frc.robot.vision.MultiVisionSources;
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
	private final IPoseEstimator poseEstimator;
	private final Superstructure superStructure;

	public Robot() {
		IGyro gyro = GyroFactory.createGyro(SwerveType.SWERVE);
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(SwerveType.SWERVE),
			ModulesFactory.create(SwerveType.SWERVE),
			gyro,
			GyroFactory.createSignals(SwerveType.SWERVE, gyro)
		);

		MultiVisionSources multiVisionSources = new MultiVisionSources(
//			new Limel
		);

		this.poseEstimator = new WPILibPoseEstimator(
			"wpilibPoseEstimator/",
			multiVisionSources,
			new OdometryValues(swerve.getConstants().kinematics(), swerve.getModulePositions(), Rotation2d.fromDegrees(0)),
			new Odometry<>(swerve.getConstants().kinematics(), Rotation2d.fromDegrees(0), swerve.getModulePositions(), new Pose2d()),
			WPILibPoseEstimator.standardDeviationsToMatrix(PoseEstimatorConstants.DEFAULT_ODOMETRY_STANDARD_DEVIATIONS),
			VecBuilder.fill(
				0.01, 0.01, 0.01
			)
		);

		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
		swerve.setStateHelper(new SwerveStateHelper(() -> Optional.of(poseEstimator.getEstimatedPose()), Optional::empty, swerve));

		this.superStructure = new Superstructure(swerve, poseEstimator);

		buildPathPlannerForAuto();
		configureBindings();
	}

	private void buildPathPlannerForAuto() {
		// Register commands...
		swerve.configPathPlanner(poseEstimator::getEstimatedPose, poseEstimator::resetPose, PathPlannerUtils.SYNCOPA_ROBOT_CONFIG);
	}

	private void configureBindings() {
		JoysticksBindings.configureBindings(this);
	}


	public Command getAutonomousCommand() {
		return new InstantCommand();
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
