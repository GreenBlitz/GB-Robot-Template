// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotManager;
import frc.robot.imu.IMUWrapper;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.robot.imu.factories.IMUFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.vision.cameras.limelight.Limelight;
import frc.robot.vision.cameras.limelight.LimelightFilters;
import frc.robot.vision.cameras.limelight.LimelightStdDevCalculations;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;
import frc.utils.brakestate.BrakeMode;
import frc.utils.brakestate.BrakeStateManager;
import org.littletonrobotics.junction.Logger;
import org.photonvision.*;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType(false);

	private final PhotonPoseEstimator photonPoseEstimator;
	private final PhotonCamera photonCamera = new PhotonCamera("testCamera");
	private final List<Limelight> limelights;

	public Robot() {
		BatteryUtil.scheduleLimiter();




		this.limelights = List.of();
		limelights.forEach(
			limelight -> limelight.setMT1StdDevsCalculation(
				LimelightStdDevCalculations.getMT1StdDevsCalculation(
					limelight,
					RobotConstants.DEFAULT_TAG_DISTANCE_FACTORS,
					RobotConstants.DEFAULT_STD_DEV_FACTORS,
					RobotConstants.DEFAULT_VISIBLE_TAGS_EXPONENTS,
					RobotConstants.DEFAULT_STD_DEV_ADDITIONS
				)
			)
		);

		photonPoseEstimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark), new Transform3d());

		configureBrakeStateChooser();
	}

	public void updateSubsystems() {

	}

	public void periodic() {
		BusChain.refreshAll();

		updateSubsystems();

		photonPoseEstimator.estimateAverageBestTargetsPose(photonCamera.getAllUnreadResults().get(0)).ifPresent((EstimatedRobotPose r) -> Logger.recordOutput("estimatorPosePhotonCameraTester",r.estimatedPose.toPose2d()));
		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public List<Limelight> getLimelights() {
		return limelights;
	}

	public PathPlannerAutoWrapper getAutonomousCommand() {
		return new PathPlannerAutoWrapper();
	}

	private void configureBrakeStateChooser() {
		SendableChooser<BrakeMode> brakeStateChooser = new SendableChooser<>();
		brakeStateChooser.setDefaultOption("Brake", BrakeMode.BRAKE);
		brakeStateChooser.addOption("Coast", BrakeMode.COAST);
		SmartDashboard.putData("BrakeState", brakeStateChooser);
		brakeStateChooser.onChange(BrakeStateManager::setBrakeMode);
	}

}
