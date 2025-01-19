// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.RobotManager;
import frc.robot.autonomous.AutonomousConstants;
import frc.constants.VisionConstants;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.signal.TimedValue;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.helpers.RobotHeadingEstimator;
import frc.robot.structures.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.swerveconstants.SwerveConstantsFactory;
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
	private final MultiAprilTagVisionSources aprilTagVisionSources;
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

		this.poseEstimator = new WPILibPoseEstimator(
			WPILibPoseEstimatorConstants.WPILIB_POSEESTIMATOR_LOGPATH,
			swerve.getKinematics(),
			swerve.getAllOdometryObservations()[0].wheelPositions(),
			WPILibPoseEstimatorConstants.INITIAL_GYRO_ANGLE
		);


//		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
		swerve.setHeadingSupplier(() -> headingEstimator.getEstimatedHeading());
		swerve.getStateHandler().setRobotPoseSupplier(poseEstimator::getEstimatedPose);

		headingEstimator = new RobotHeadingEstimator(swerve.getGyroAbsoluteYaw(), 0.0001);

		this.aprilTagVisionSources = new MultiAprilTagVisionSources(
			VisionConstants.MULTI_VISION_SOURCES_LOGPATH,
			() -> (headingEstimator.getEstimatedHeading()),
			() -> Rotation2d.fromDegrees(0),
			VisionConstants.DEFAULT_VISION_POSEESTIMATING_SOURCES
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
		headingEstimator.updateGyroAngle(swerve.getGyroAbsoluteYaw(), TimeUtils.getCurrentTimeSeconds());
		List<TimedValue<Rotation2d>> headingAndTime = aprilTagVisionSources.getRawRobotHeadings();
		if (!headingAndTime.isEmpty()) {
			Logger.recordOutput("Robot Heading", headingAndTime.get(0).value());
			headingEstimator.updateVisionHeading(headingAndTime.get(0).value(), headingAndTime.get(0).timestamp());
//			headingEstimator.updateVisionHeading(headingAndTime.get(0).getFirst(), TimeUtils.getCurrentTimeSeconds());
		}
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

	public MultiAprilTagVisionSources getAprilTagVisionSources() {
		return aprilTagVisionSources;
	}

	public RobotHeadingEstimator getHeadingEstimator() {
		return headingEstimator;
	}

}
