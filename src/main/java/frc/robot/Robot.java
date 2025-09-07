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
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.vision.DetectedObjectType;
import frc.robot.vision.cameras.limelight.Limelight;
import frc.robot.vision.cameras.limelight.LimelightFilters;
import frc.robot.vision.cameras.limelight.LimelightPipeline;
import frc.robot.vision.cameras.limelight.LimelightStdDevCalculations;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.poseestimator.helpers.robotheadingestimator.RobotHeadingEstimator;
import frc.robot.poseestimator.helpers.robotheadingestimator.RobotHeadingEstimatorConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.utils.TimedValue;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;
import frc.utils.math.StandardDeviations2D;
import frc.utils.time.TimeUtil;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final Swerve swerve;
	private final IPoseEstimator poseEstimator;
	private final Limelight limelightFour;
	private final Limelight limelightThreeGB;
	private final Limelight limelightObjectDetector;
	public final RobotHeadingEstimator headingEstimator;

	public Robot() {
		BatteryUtil.scheduleLimiter();

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
			swerve.getModules().getWheelPositions(0),
			swerve.getGyroAbsoluteYaw()
		);

		this.headingEstimator = new RobotHeadingEstimator(
			RobotHeadingEstimatorConstants.DEFAULT_HEADING_ESTIMATOR_LOGPATH,
			new Rotation2d(),
			new Rotation2d(),
			RobotHeadingEstimatorConstants.DEFAULT_GYRO_STANDARD_DEVIATION
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
		limelightFour.setMT1PoseFilter(LimelightFilters.megaTag1Filter(limelightFour, new Translation2d(0.1, 0.1)));
		limelightFour.setMT2PoseFilter(
			LimelightFilters.megaTag2Filter(
				limelightFour,
				headingEstimator::getEstimatedHeadingAtTimestamp,
				new Translation2d(0.1, 0.1),
				Rotation2d.fromDegrees(2)
			)
		);
		limelightFour.setMT1StdDevsCalculation(
			LimelightStdDevCalculations.getMT1StdDevsCalculation(
				limelightFour,
				new StandardDeviations2D(0.0001, 0.0001, 0.0001),
				new StandardDeviations2D(0.001, 0.001, 0.001)
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
		limelightThreeGB.setMT1PoseFilter(LimelightFilters.megaTag1Filter(limelightThreeGB, new Translation2d(0.1, 0.1)));
		limelightThreeGB.setMT2PoseFilter(
			LimelightFilters.megaTag2Filter(
				limelightThreeGB,
				headingEstimator::getEstimatedHeadingAtTimestamp,
				new Translation2d(0.1, 0.1),
				Rotation2d.fromDegrees(2)
			)
		);
		limelightThreeGB.setMT1StdDevsCalculation(
			LimelightStdDevCalculations.getMT1StdDevsCalculation(
				limelightThreeGB,
				new StandardDeviations2D(0.0001, 0.0001, 0.0001),
				new StandardDeviations2D(0.001, 0.001, 0.001)
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
		limelightObjectDetector
			.setDetectedObjectFilter(LimelightFilters.detectedObjectFilter(limelightObjectDetector, DetectedObjectType.ALGAE));

//		swerve.setHeadingSupplier(
//			ROBOT_TYPE.isSimulation() ? () -> poseEstimator.getEstimatedPose().getRotation() : () -> headingEstimator.getEstimatedHeading()
//		);
		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
	}

	public void periodic() {
		BusChain.refreshAll();

		swerve.update();
		poseEstimator.updateOdometry(swerve.getAllOdometryData());
		headingEstimator.updateGyroAngle(new TimedValue<>(swerve.getGyroAbsoluteYaw(), TimeUtil.getCurrentTimeSeconds()));

		limelightFour.updateMT1();
		limelightThreeGB.updateMT1();

		limelightFour.getIndependentRobotPose()
			.ifPresent(
				robotPoseObservation -> headingEstimator.updateVisionIfGyroOffsetIsNotCalibrated(
					new TimedValue<>(robotPoseObservation.robotPose().getRotation(), robotPoseObservation.timestampSeconds()),
					RobotHeadingEstimatorConstants.DEFAULT_VISION_STANDARD_DEVIATION,
					RobotHeadingEstimatorConstants.MAXIMUM_STANDARD_DEVIATION_TOLERANCE
				)
			);
		limelightThreeGB.getIndependentRobotPose()
			.ifPresent(
				robotPoseObservation -> headingEstimator.updateVisionIfGyroOffsetIsNotCalibrated(
					new TimedValue<>(robotPoseObservation.robotPose().getRotation(), robotPoseObservation.timestampSeconds()),
					RobotHeadingEstimatorConstants.DEFAULT_VISION_STANDARD_DEVIATION,
					RobotHeadingEstimatorConstants.MAXIMUM_STANDARD_DEVIATION_TOLERANCE
				)
			);

		limelightFour.setRobotOrientation(headingEstimator.getEstimatedHeading());
		limelightThreeGB.setRobotOrientation(headingEstimator.getEstimatedHeading());

		limelightFour.updateMT2();
		limelightThreeGB.updateMT2();

		limelightFour.getOrientationRequiringRobotPose().ifPresent(poseEstimator::updateVision);
		limelightThreeGB.getOrientationRequiringRobotPose().ifPresent(poseEstimator::updateVision);

		limelightFour.log();
		limelightThreeGB.log();
		headingEstimator.log();

		limelightObjectDetector.updateObjectDetection();
		limelightObjectDetector.log();

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

	public IPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

}
