// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.RobotManager;
import frc.constants.VisionConstants;
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.poseestimation.PoseEstimator;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.structures.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.swerveconstants.SwerveConstantsFactory;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.multivisionsources.MultiAprilTagVisionSources;
import frc.robot.vision.sources.limelights.DynamicSwitchingLimelight;
import frc.robot.vision.sources.limelights.LimelightFactory;
import frc.utils.Filter;
import frc.utils.auto.AutonomousChooser;
import frc.utils.auto.PathPlannerUtils;
import frc.utils.battery.BatteryUtils;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final Swerve swerve;
	private final IPoseEstimator poseEstimator;
	private final MultiAprilTagVisionSources multiAprilTagVisionSources;
	private final Superstructure superStructure;

	private AutonomousChooser autonomousChooser;

	public Robot() {
		BatteryUtils.scheduleLimiter();

		IGyro gyro = GyroFactory.createGyro(RobotConstants.SUBSYSTEM_LOG_PREFIX + "Swerve");
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(RobotConstants.SUBSYSTEM_LOG_PREFIX + "Swerve"),
			ModulesFactory.create(RobotConstants.SUBSYSTEM_LOG_PREFIX + "Swerve"),
			gyro,
			GyroFactory.createSignals(gyro)
		);

		this.poseEstimator = new WPILibPoseEstimatorWrapper("poseEstimator/",
			swerve.getKinematics(),
			swerve.getAllOdometryObservations()[0].wheelPositions(),
			swerve.getGyroAbsoluteYaw()
		);

		this.multiAprilTagVisionSources = new MultiAprilTagVisionSources(
			VisionConstants.MULTI_VISION_SOURCES_LOGPATH,
			() -> poseEstimator.getEstimatedPose().getRotation(),
			new DynamicSwitchingLimelight(
				false,
				"limelight-back",
				VisionConstants.VISION_SOURCE_LOGPATH_ADDITION,
				"CameraOnTheRobot",
				Filter.nonFilteringFilter()
			)
		);

		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
		swerve.getStateHandler().setRobotPoseSupplier(poseEstimator::getEstimatedPose);

		this.superStructure = new Superstructure(swerve, poseEstimator);

		buildPathPlannerForAuto();
	}


	private void buildPathPlannerForAuto() {
		// Register commands...
		swerve.configPathPlanner(
			poseEstimator::getEstimatedPose,
			poseEstimator::resetPose,
			PathPlannerUtils.getGuiRobotConfig().orElse(AutonomousConstants.SYNCOPA_ROBOT_CONFIG)
		);
		autonomousChooser = new AutonomousChooser("AutonomousChooser");
	}


	public void periodic() {
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		superStructure.periodic();
		poseEstimator.updateVision(multiAprilTagVisionSources.getFilteredVisionData());
		CommandScheduler.getInstance().run(); // Should be last
	}

	public Command getAutonomousCommand() {
		return autonomousChooser.getChosenValue();
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
