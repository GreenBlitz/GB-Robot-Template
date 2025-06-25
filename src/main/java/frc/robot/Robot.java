// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotManager;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.newvision.cameras.limelight.Limelight;
import frc.robot.newvision.cameras.limelight.LimelightPipeline;
import frc.robot.newvision.cameras.limelight.LimelightStandardDeviationsCalculations;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.poseestimator.helpers.robotheadingestimator.RobotHeadingEstimator;
import frc.robot.poseestimator.helpers.robotheadingestimator.RobotHeadingEstimatorConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
//import frc.robot.vision.VisionConstants;
//import frc.robot.vision.VisionFilters;
//import frc.robot.vision.multivisionsources.MultiAprilTagVisionSources;
import frc.utils.TimedValue;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;
import frc.utils.time.TimeUtil;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final Swerve swerve;
	private final IPoseEstimator poseEstimator;
	private final Limelight limelight;
//	private final MultiAprilTagVisionSources visionSources;
	private final RobotHeadingEstimator headingEstimator;

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

		this.limelight = new Limelight(
			"limelight-left",
			"NewVision",
			new Pose3d(),
			LimelightPipeline.APRIL_TAG,
			LimelightStandardDeviationsCalculations.averageTagDistanceParabola(
				MatBuilder.fill(Nat.N3(), Nat.N1(), 0.0001, 0.0001, 0.0001),
				MatBuilder.fill(Nat.N3(), Nat.N1(), 0.001, 0.001, 0.001)
			),
			LimelightStandardDeviationsCalculations.averageTagDistanceParabola(
				MatBuilder.fill(Nat.N3(), Nat.N1(), 0.0001, 0.0001, 0.9999),
				MatBuilder.fill(Nat.N3(), Nat.N1(), 0.001, 0.001, 0.9999)
			)
		);

//		this.visionSources = new MultiAprilTagVisionSources(
//			VisionConstants.MULTI_VISION_SOURCES_LOGPATH,
//			headingEstimator::getEstimatedHeading,
//			true,
//			VisionConstants.VISION_SOURCES
//		);
//
//		visionSources.applyFunctionOnAllFilters(
//			filter -> filter.and(
//				data -> VisionFilters
//					.isYawAtAngleForMegaTag2(
//						() -> headingEstimator.getEstimatedHeadingAtTimestamp(data.getTimestamp()),
//						VisionConstants.YAW_FILTER_TOLERANCE
//					)
//					.and(VisionFilters.isYawAngleNotZero())
//					.apply(data)
//			)
//		);

		swerve.setHeadingSupplier(
			ROBOT_TYPE.isSimulation() ? () -> poseEstimator.getEstimatedPose().getRotation() : () -> headingEstimator.getEstimatedHeading()
		);
	}

	public void periodic() {
		BusChain.refreshAll();

		swerve.update();
		poseEstimator.updateOdometry(swerve.getAllOdometryData());
//		poseEstimator.updateVision(visionSources.getFilteredVisionData());
		headingEstimator.updateGyroAngle(new TimedValue<>(swerve.getGyroAbsoluteYaw(), TimeUtil.getCurrentTimeSeconds()));
//		for (TimedValue<Rotation2d> headingData : visionSources.getFilteredRobotHeading()) {
//			headingEstimator.updateVisionIfGyroOffsetIsNotCalibrated(
//				headingData,
//				RobotHeadingEstimatorConstants.DEFAULT_VISION_STANDARD_DEVIATION,
//				RobotHeadingEstimatorConstants.MAXIMUM_STANDARD_DEVIATION_TOLERANCE
//			);
//		}
		limelight.updateMegaTag1();
		headingEstimator.updateVisionIfGyroOffsetIsNotCalibrated(
			limelight.getIndependentRobotPose(),
			RobotHeadingEstimatorConstants.DEFAULT_VISION_STANDARD_DEVIATION,
			RobotHeadingEstimatorConstants.MAXIMUM_STANDARD_DEVIATION_TOLERANCE
		);
		limelight.setRobotOrientation(headingEstimator.getEstimatedHeading());
		limelight.updateMegaTag2();
		poseEstimator.updateVision(List.of(limelight.getIndependentRobotPose(), limelight.getOrientationRequiringRobotPose()));
		limelight.log();
		headingEstimator.log();

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

}
