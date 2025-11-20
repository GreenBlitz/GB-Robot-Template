// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotManager;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.statemachine.RobotCommander;
import frc.robot.subsystems.swerve.factories.imu.IMUFactory;
import frc.robot.vision.cameras.limelight.*;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;
import frc.utils.math.StandardDeviations2D;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType(false);

	private final Swerve swerve;
	private final IPoseEstimator poseEstimator;
	private final Limelight limelightFour;
	private final Limelight limelightThreeGB;
	private final Limelight limelightObjectDetector;
	private final RobotCommander robotCommander;

	public Robot() {
		BatteryUtil.scheduleLimiter();

		IIMU imu = IMUFactory.createIMU(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve");
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			ModulesFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			imu,
			IMUFactory.createSignals(imu)
		);

		this.poseEstimator = new WPILibPoseEstimatorWrapper(
			WPILibPoseEstimatorConstants.WPILIB_POSEESTIMATOR_LOGPATH,
			swerve.getKinematics(),
			swerve.getModules().getWheelPositions(0),
			swerve.getGyroAbsoluteYaw()
		);

		this.limelightFour = new Limelight(
			"limelight-left",
			"NewVision",
			new Pose3d(
				new Translation3d(0.215, -0.11, 0.508),
				new Rotation3d(Units.Degrees.of(-8.06180374425555), Units.Degrees.of(-27.07784559039065), Units.Degrees.of(-22.52372569716833))
			),
			LimelightPipeline.APRIL_TAG
		);
		limelightFour.setMT1PoseFilter(
			LimelightFilters.megaTag1Filter(
				limelightFour,
				timestamp -> poseEstimator.getEstimatedPoseAtTimestamp(timestamp).getRotation(),
				poseEstimator::isIMUOffsetCalibrated,
				new Translation2d(0.1, 0.1),
				Rotation2d.fromDegrees(10)
			)
		);
		limelightFour.setMT2PoseFilter(
			LimelightFilters.megaTag2Filter(
				limelightFour,
				timestamp -> poseEstimator.getEstimatedPoseAtTimestamp(timestamp).getRotation(),
				poseEstimator::isIMUOffsetCalibrated,
				new Translation2d(0.1, 0.1),
				Rotation2d.fromDegrees(2)
			)
		);
		limelightFour.setMT1StdDevsCalculation(
			LimelightStdDevCalculations.getMT1StdDevsCalculation(
				limelightFour,
				new StandardDeviations2D(0.5),
				new StandardDeviations2D(0.05),
				new StandardDeviations2D(-0.02)
			)
		);
		limelightFour.setMT2StdDevsCalculation(
			LimelightStdDevCalculations.getMT2StdDevsCalculation(
				limelightFour,
				new StandardDeviations2D(0.0001, 0.0001, 0.9999),
				new StandardDeviations2D(0.001, 0.001, 0.9999)
			)
		);

		this.limelightThreeGB = new Limelight(
			"limelight",
			"NewVision",
			new Pose3d(
				new Translation3d(0.2022, 0.13, 0.508),
				new Rotation3d(Units.Degrees.of(10.612258493096334), Units.Degrees.of(-27.18966371065684), Units.Degrees.of(20.10328620400214))
			),
			LimelightPipeline.APRIL_TAG
		);
		limelightThreeGB.setMT1PoseFilter(
			LimelightFilters.megaTag1Filter(
				limelightThreeGB,
				timestamp -> poseEstimator.getEstimatedPoseAtTimestamp(timestamp).getRotation(),
				poseEstimator::isIMUOffsetCalibrated,
				new Translation2d(0.1, 0.1),
				Rotation2d.fromDegrees(10)
			)
		);
		limelightThreeGB.setMT2PoseFilter(
			LimelightFilters.megaTag2Filter(
				limelightThreeGB,
				timestamp -> poseEstimator.getEstimatedPoseAtTimestamp(timestamp).getRotation(),
				poseEstimator::isIMUOffsetCalibrated,
				new Translation2d(0.1, 0.1),
				Rotation2d.fromDegrees(2)
			)
		);
		limelightThreeGB.setMT1StdDevsCalculation(
			LimelightStdDevCalculations.getMT1StdDevsCalculation(
				limelightThreeGB,
				new StandardDeviations2D(0.5),
				new StandardDeviations2D(0.05),
				new StandardDeviations2D(-0.02)
			)
		);
		limelightThreeGB.setMT2StdDevsCalculation(
			LimelightStdDevCalculations.getMT2StdDevsCalculation(
				limelightThreeGB,
				new StandardDeviations2D(0.0001, 0.0001, 0.9999),
				new StandardDeviations2D(0.001, 0.001, 0.9999)
			)
		);

		limelightObjectDetector = new Limelight(
			"limelight-object",
			"NewVision",
			new Pose3d(
				new Translation3d(-0.08, 0.23, 0.865),
				new Rotation3d(Units.Degrees.of(0), Units.Degrees.of(-27), Units.Degrees.of(-176.67))
			),
			LimelightPipeline.OBJECT_DETECTION
		);

		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());

		this.robotCommander = new RobotCommander("StateMachine/RobotCommander", this);
	}

	public void periodic() {
		BusChain.refreshAll();

		swerve.update();
		poseEstimator.updateOdometry(swerve.getAllOdometryData());

		limelightFour.updateMT1();
		limelightThreeGB.updateMT1();

		limelightFour.setRobotOrientation(poseEstimator.getEstimatedPose().getRotation());
		limelightThreeGB.setRobotOrientation(poseEstimator.getEstimatedPose().getRotation());

		limelightFour.updateMT2();
		limelightThreeGB.updateMT2();

		limelightFour.getIndependentRobotPose().ifPresent(poseEstimator::updateVision);
		limelightThreeGB.getIndependentRobotPose().ifPresent(poseEstimator::updateVision);

		limelightObjectDetector.updateObjectDetection();
		
		poseEstimator.log();

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public PathPlannerAutoWrapper getAutonomousCommand() {
		return new PathPlannerAutoWrapper();
	}

	public Swerve getSwerve() {
		return swerve;
	}

	public RobotCommander getRobotCommander() {
		return robotCommander;
	}

	public IPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

}
