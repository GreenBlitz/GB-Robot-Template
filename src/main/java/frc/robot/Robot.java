// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.RobotConstants;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.poseestimator.GBPoseEstimator;
import frc.robot.poseestimator.OdometryValues;
import frc.robot.poseestimator.PoseEstimatorConstants;
import frc.robot.structures.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.swerveconstants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.swervestatehelpers.SwerveStateHelper;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.multivisionsources.MultiPoseEstimatingVisionSources;
import frc.robot.vision.sources.simulationsource.SimulatedSource;
import frc.utils.auto.PathPlannerUtils;

import java.util.Optional;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private static final boolean IS_MAPLE = true;

	private final Swerve swerve;
	private final GBPoseEstimator poseEstimator;
	private final Superstructure superStructure;

	public Robot() {
		IGyro gyro = GyroFactory.createGyro(SwerveType.SWERVE, IS_MAPLE);
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(SwerveType.SWERVE),
			ModulesFactory.create(SwerveType.SWERVE, IS_MAPLE),
			gyro,
			GyroFactory.createSignals(SwerveType.SWERVE, gyro)
		);

		this.poseEstimator = new GBPoseEstimator(
			PoseEstimatorConstants.LOG_PATH,
			new MultiPoseEstimatingVisionSources(
				new SimulatedSource(
					"limelight-front",
					() -> swerve.getSimulatedPose(),
					new Pose3d(),
					VisionConstants.LIMELIGHT_3_SIMULATED_SOURCE_CONFIGURATION
				)
			),
			VisionConstants.DEFAULT_VISION_FILTERER_CONFIG,
			new OdometryValues(swerve.getConstants().kinematics(), swerve.getModules().getWheelsPositions(0), swerve.getAbsoluteHeading()),
			PoseEstimatorConstants.DEFAULT_ODOMETRY_STANDARD_DEVIATIONS
		);

		swerve.applyPhysicsSimulation(
			RobotConstants.ROBOT_MASS_WIDTH_BUMPERS_KG,
			RobotConstants.BUMPER_WIDTH_METERS,
			RobotConstants.BUMPER_LENGTH_METERS,
			new Pose2d(2, 5, Rotation2d.fromDegrees(10))
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

	public GBPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

}
