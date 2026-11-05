// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.subsystems.HoodConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.factories.imu.IMUFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.vision.cameras.limelight.Limelight;
import frc.robot.vision.cameras.limelight.LimelightFilters;
import frc.robot.vision.cameras.limelight.LimelightPipeline;
import frc.robot.vision.cameras.limelight.LimelightStdDevCalculations;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;
import frc.robot.hardware.interfaces.IIMU;
import frc.utils.brakestate.BrakeMode;
import frc.utils.brakestate.BrakeStateManager;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType(false);

	private final Swerve swerve;
	public final Limelight limelight;
	public final Arm arm;
	public final Arm motionMagicArm;
	public final Arm dynamicMotionMagicArm;
	private final IPoseEstimator poseEstimator;
	private final List<Limelight> limelights;

	public Robot() {
		BatteryUtil.scheduleLimiter();

		IIMU imu = IMUFactory.createIMU(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve");
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			ModulesFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			imu,
			IMUFactory.createSignals(imu)
		);

		this.limelight = new Limelight(
			"limelight-front",
			"Vision",
			new Pose3d(
				new Translation3d(0.297, -0.143, 0.361),
				new Rotation3d(Math.toRadians(-0.18), Math.toRadians(27.38), Math.toRadians(-0.35))
			),
			LimelightPipeline.APRIL_TAG
		);

		this.arm = HoodConstants.createArm();
		this.dynamicMotionMagicArm = HoodConstants.createDynamicMotionMagicArm();
		this.motionMagicArm = HoodConstants.createMotionMagicArm();

		BrakeStateManager.add(() -> swerve.getModules().setBrake(true), () -> swerve.getModules().setBrake(false));
		this.poseEstimator = new WPILibPoseEstimatorWrapper(
			WPILibPoseEstimatorConstants.WPILIB_POSEESTIMATOR_LOGPATH,
			swerve.getKinematics(),
			swerve.getModules().getWheelPositions(0),
			swerve.getModules().getCurrentStates(),
			swerve.getOrientationFromIMU(),
			swerve.getIMUAccelerationG(),
			swerve.getIMUAbsoluteYaw().getTimestamp()
		);

		this.limelights = List.of(limelight);
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
		limelights.forEach(
			limelight -> limelight.setMT1PoseFilter(
				LimelightFilters.megaTag1Filter(
					limelight,
					timestamp -> poseEstimator.getEstimatedPoseAtTimestamp(timestamp).map(Pose2d::getRotation),
					poseEstimator::isIMUOffsetCalibrated,
					LimelightFilters.DEFAULT_IN_FIELD_TOLERANCE_METERS,
					LimelightFilters.DEFAULT_YAW_AT_ANGLE_TOLERANCE
				)
			)
		);

		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());

		configureBrakeStateChooser();
	}

	public void updateSubsystems() {
		swerve.update();
		arm.update();
		motionMagicArm.update();
		arm.update();
	}

	public void periodic() {
		BusChain.refreshAll();

		updateSubsystems();
		poseEstimator.updateOdometry(swerve.getAllOdometryData());

		getLimelights().forEach(Limelight::updateHardwareInputs);
		getLimelights().forEach(Limelight::updateMT1);
		getLimelights().forEach(limelight -> limelight.getIndependentRobotPose().ifPresent(poseEstimator::updateVision));

		poseEstimator.log();

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public IPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

	public Swerve getSwerve() {
		return swerve;
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
