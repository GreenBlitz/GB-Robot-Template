// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.poseestimator.GBPoseEstimator;
import frc.robot.poseestimator.PoseEstimatorConstants;
import frc.robot.poseestimator.VisionDenoiser;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.swerveconstants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.swervestatehelpers.SwerveStateHelper;
import frc.robot.vision.limelights.LimeLightConstants;
import frc.robot.vision.limelights.LimelightFilterer;
import frc.robot.vision.limelights.LimelightFiltererConfig;
import frc.robot.vision.limelights.MultiLimelights;
import frc.utils.auto.AutonomousChooser;

import java.util.Arrays;
import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private AutonomousChooser autonomousChooser;

	private final Swerve swerve;
	private final GBPoseEstimator poseEstimator;
	private final LimelightFilterer limelightFilterer;
	private final MultiLimelights multiLimelights;

	public Robot() {
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(SwerveType.SWERVE),
			ModulesFactory.create(SwerveType.SWERVE),
			GyroFactory.create(SwerveType.SWERVE)
		);
		swerve.updateStatus();

		this.multiLimelights = new MultiLimelights(LimeLightConstants.LIMELIGHT_NAMES, "limelightsHardware/");
		this.limelightFilterer = new LimelightFilterer(
			new LimelightFiltererConfig("limelightfilterer/", LimeLightConstants.DEFAULT_LIMELIGHT_FILTERS_TOLERANCES),
			multiLimelights
		);
		this.poseEstimator = new GBPoseEstimator(
			swerve::setHeading,
			"PoseEstimator/",
			limelightFilterer,
			swerve.getConstants().kinematics(),
			swerve.getModules().getWheelsPositions(0),
			swerve.getAbsoluteHeading(),
			PoseEstimatorConstants.DEFAULT_ODOMETRY_STANDARD_DEVIATIONS,
			new VisionDenoiser(20)
		);
		limelightFilterer.setEstimatedPoseAtTimestampFunction(poseEstimator::getEstimatedPoseAtTimeStamp);

		swerve.configPathPlanner(poseEstimator::getEstimatedPose, pose2d -> {});
		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
		swerve.setStateHelper(new SwerveStateHelper(() -> Optional.of(poseEstimator.getEstimatedPose()), Optional::empty, swerve));

		configPathPlanner();
		configureBindings();
	}

	public void periodic() {
		swerve.updateStatus();
		poseEstimator.updateVision(limelightFilterer.getFilteredVisionObservations());
		poseEstimator.updateOdometry(Arrays.asList(swerve.getAllOdometryObservations()));
	}

	private void configPathPlanner() {
		// Register commands...
		swerve.configPathPlanner(poseEstimator::getEstimatedPose, poseEstimator::resetPose);
		autonomousChooser = new AutonomousChooser("Autonomous Chooser");
	}

	private void configureBindings() {
		JoysticksBindings.configureBindings(this);
	}


	public Command getAutonomousCommand() {
		return autonomousChooser.getChosenValue();
	}

	public Swerve getSwerve() {
		return swerve;
	}

	public GBPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

}
