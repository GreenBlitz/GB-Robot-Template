// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.RobotManager;
import frc.constants.RobotHeadingEstimatorConstants;
import frc.constants.VisionConstants;
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.poseestimator.helpers.RobotHeadingEstimator;
import frc.robot.structures.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.multivisionsources.MultiAprilTagVisionSources;
import frc.robot.vision.sources.VisionSource;
import frc.robot.vision.sources.limelights.DynamicSwitchingLimelight;
import frc.utils.AngleUnit;
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
	private final Superstructure superStructure;

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
			WPILibPoseEstimatorConstants.WPILIB_POSEESTIMATOR_LOGPATH,
			swerve.getKinematics(),
			swerve.getAllOdometryData()[0].wheelPositions(),
			swerve.getAllOdometryData()[0].gyroAngle().orElse(new Rotation2d())
		);

		RobotHeadingEstimator headingEstimator = new RobotHeadingEstimator(
			RobotHeadingEstimatorConstants.DEFAULT_HEADING_ESTIMATOR_LOGPATH,
			swerve.getAllOdometryData()[0].gyroAngle().orElse(new Rotation2d()),
			swerve.getAllOdometryData()[0].gyroAngle().orElse(new Rotation2d()),
			RobotHeadingEstimatorConstants.DEFAULT_GYRO_STANDARD_DEVIATION
		);

		MultiAprilTagVisionSources multiAprilTagVisionSources = new MultiAprilTagVisionSources(
			VisionConstants.MULTI_VISION_SOURCES_LOGPATH,
			headingEstimator::getEstimatedHeading,
			true,
			new DynamicSwitchingLimelight(
				true,
				"limelight-back",
				VisionConstants.MULTI_VISION_SOURCES_LOGPATH,
				"limelightForPoseEstimating",
				VisionConstants.DEFAULT_VISION_FILTER,
				new Pose3d(new Translation3d(0.07, -0.24, 0.54), AngleUnit.DEGREES.toRotation3d(0, -16, 0))
			)
		);

		swerve.setHeadingSupplier(headingEstimator::getEstimatedHeading);
		swerve.getStateHandler().setRobotPoseSupplier(poseEstimator::getEstimatedPose);

		this.superStructure = new Superstructure(swerve, poseEstimator, headingEstimator, multiAprilTagVisionSources);

		buildPathPlannerForAuto();
	}


	private void buildPathPlannerForAuto() {
		// Register commands...
		swerve.configPathPlanner(
			poseEstimator::getEstimatedPose,
			poseEstimator::resetPose,
			PathPlannerUtils.getGuiRobotConfig().orElse(AutonomousConstants.SYNCOPA_ROBOT_CONFIG)
		);
	}


	public void periodic() {
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		superStructure.periodic();
		CommandScheduler.getInstance().run(); // Should be last
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
