// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotManager;
import frc.constants.RobotHeadingEstimatorConstants;
import frc.robot.autonomous.AutonomousConstants;
import frc.constants.VisionConstants;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.signal.TimedValue;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.poseestimator.helpers.RobotHeadingEstimator;
import frc.robot.structures.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.swerveconstants.SwerveConstantsFactory;
import frc.robot.vision.data.HeadingData;
import frc.utils.auto.AutonomousChooser;
import frc.robot.vision.multivisionsources.MultiAprilTagVisionSources;
import frc.utils.auto.PathPlannerUtils;
import frc.robot.hardware.phoenix6.BusChain;
import frc.utils.battery.BatteryUtils;
import frc.utils.time.TimeUtils;
import org.littletonrobotics.junction.Logger;

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
	private final IPoseEstimator poseEstimator2;
	private final IPoseEstimator poseEstimator3;
	private final MultiAprilTagVisionSources aprilTagVisionSources;
	private final MultiAprilTagVisionSources aprilTagVisionSources2;
	private final MultiAprilTagVisionSources aprilTagVisionSources3;
	private final Superstructure superStructure;
	private RobotHeadingEstimator headingEstimator = null;

	private AutonomousChooser autonomousChooser;

	public Robot() {
		BatteryUtils.scheduleLimiter();

		IGyro gyro = GyroFactory.createGyro(RobotConstants.SUBSYSTEM_LOG_PREFIX + "Swerve/");
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(RobotConstants.SUBSYSTEM_LOG_PREFIX + "Swerve/"),
			ModulesFactory.create(RobotConstants.SUBSYSTEM_LOG_PREFIX + "Swerve/"),
			gyro,
			GyroFactory.createSignals(gyro)
		);

		this.poseEstimator = new WPILibPoseEstimatorWrapper(
			WPILibPoseEstimatorConstants.WPILIB_POSEESTIMATOR_LOGPATH,
			swerve.getKinematics(),
			swerve.getAllOdometryObservations()[0].wheelPositions(),
			WPILibPoseEstimatorConstants.INITIAL_GYRO_ANGLE
		);

		this.poseEstimator2 = new WPILibPoseEstimatorWrapper(
			WPILibPoseEstimatorConstants.WPILIB_POSEESTIMATOR_LOGPATH
				.substring(0, WPILibPoseEstimatorConstants.WPILIB_POSEESTIMATOR_LOGPATH.length() - 1)
				+ "2/",
			swerve.getKinematics(),
			swerve.getAllOdometryObservations()[0].wheelPositions(),
			WPILibPoseEstimatorConstants.INITIAL_GYRO_ANGLE
		);

		this.poseEstimator3 = new WPILibPoseEstimatorWrapper(
			WPILibPoseEstimatorConstants.WPILIB_POSEESTIMATOR_LOGPATH
				.substring(0, WPILibPoseEstimatorConstants.WPILIB_POSEESTIMATOR_LOGPATH.length() - 1)
				+ "3/",
			swerve.getKinematics(),
			swerve.getAllOdometryObservations()[0].wheelPositions(),
			WPILibPoseEstimatorConstants.INITIAL_GYRO_ANGLE
		);


//		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
		swerve.setHeadingSupplier(() -> headingEstimator.getEstimatedHeading().plus(Rotation2d.fromDegrees(150)));
		swerve.getStateHandler().setRobotPoseSupplier(poseEstimator::getEstimatedPose);

		headingEstimator = new RobotHeadingEstimator(swerve.getGyroAbsoluteYaw(), swerve.getGyroAbsoluteYaw(), 0.0001);

		this.aprilTagVisionSources = new MultiAprilTagVisionSources(
			VisionConstants.MULTI_VISION_SOURCES_LOGPATH,
			() -> (headingEstimator.getEstimatedHeading()),
			() -> Rotation2d.fromDegrees(0),
			VisionConstants.DEFAULT_VISION_POSEESTIMATING_SOURCES
		);

		this.aprilTagVisionSources2 = new MultiAprilTagVisionSources(
			VisionConstants.MULTI_VISION_SOURCES_LOGPATH + "2",
			() -> (headingEstimator.getEstimatedHeading()),
			() -> Rotation2d.fromDegrees(0),
			VisionConstants.DEFAULT_VISION_POSEESTIMATING_SOURCES2
		);

		this.aprilTagVisionSources3 = new MultiAprilTagVisionSources(
			VisionConstants.MULTI_VISION_SOURCES_LOGPATH + "3",
			() -> (headingEstimator.getEstimatedHeading()),
			() -> Rotation2d.fromDegrees(0),
			VisionConstants.DEFAULT_VISION_POSEESTIMATING_SOURCES3
		);

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
		// testing:
//		swerve.configPathPlanner(() -> new Pose2d(poseEstimator.getEstimatedPose().getTranslation(), headingEstimator.getEstimatedHeading()), pose -> {
//			poseEstimator.resetPose(pose);
//			headingEstimator.reset(pose.getRotation());
//		}, PathPlannerUtils.SYNCOPA_ROBOT_CONFIG);
//		autonomousChooser = new AutonomousChooser("autonomousChooser");
	}


	public void periodic() {
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		superStructure.periodic();
		aprilTagVisionSources.log();
		aprilTagVisionSources2.log();
		aprilTagVisionSources3.log();
		headingEstimator.updateGyroAngle(new HeadingData(swerve.getGyroAbsoluteYaw(), TimeUtils.getCurrentTimeSeconds()));
		List<TimedValue<Rotation2d>> headingAndTime = aprilTagVisionSources.getRawRobotHeadings();
		if (!headingAndTime.isEmpty()) {
			Logger.recordOutput("Robot Heading", headingAndTime.get(0).value());
			headingEstimator.updateVisionIfNotCalibrated(new HeadingData(headingAndTime.get(0).value(), headingAndTime.get(0).timestamp()), RobotHeadingEstimatorConstants.DEFAULT_VISION_STANDARD_DEVIATION, 0.001);
//			headingEstimator.updateVisionHeading(headingAndTime.get(0).value(), headingAndTime.get(0).timestamp());
//			headingEstimator.updateVisionHeading(headingAndTime.get(0).getFirst(), TimeUtils.getCurrentTimeSeconds());
		}
		headingEstimator.periodic();
		Logger.recordOutput("Robot Heading By Estimator", new Pose2d(new Translation2d(0, 0), headingEstimator.getEstimatedHeading()));
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

	public IPoseEstimator[] getPoseEstimators() {
		return new IPoseEstimator[] {poseEstimator, poseEstimator2, poseEstimator3};
	}

	public MultiAprilTagVisionSources[] getAprilTagVisionSources() {
		return new MultiAprilTagVisionSources[] {aprilTagVisionSources, aprilTagVisionSources2, aprilTagVisionSources3};
	}

	public RobotHeadingEstimator getHeadingEstimator() {
		return headingEstimator;
	}

}
