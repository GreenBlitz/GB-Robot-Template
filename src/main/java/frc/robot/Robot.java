// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotManager;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.poseestimator.helpers.robotheadingestimator.RobotHeadingEstimator;
import frc.robot.poseestimator.helpers.robotheadingestimator.RobotHeadingEstimatorConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.modules.drive.KrakenX60DriveBuilder;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.VisionFilters;
import frc.robot.vision.multivisionsources.MultiAprilTagVisionSources;
import frc.utils.TimedValue;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.battery.BatteryUtil;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final Swerve swerve;
	private final IPoseEstimator poseEstimator;
	private final MultiAprilTagVisionSources visionSources;
	private final RobotHeadingEstimator headingEstimator;

	public Optional<Translation2d> object;

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

		this.visionSources = new MultiAprilTagVisionSources(
			VisionConstants.MULTI_VISION_SOURCES_LOGPATH,
			headingEstimator::getEstimatedHeading,
			true,
			VisionConstants.VISION_SOURCES
		);

		visionSources.applyFunctionOnAllFilters(
			filter -> filter.and(
				data -> VisionFilters
					.isYawAtAngleForMegaTag2(
						() -> headingEstimator.getEstimatedHeadingAtTimestamp(data.getTimestamp()),
						VisionConstants.YAW_FILTER_TOLERANCE
					)
					.and(VisionFilters.isYawAngleNotZero())
					.apply(data)
			)
		);

		swerve.setHeadingSupplier(
			ROBOT_TYPE.isSimulation() ? () -> poseEstimator.getEstimatedPose().getRotation() : () -> headingEstimator.getEstimatedHeading()
		);

		object = Optional.empty();

		configureAuto();
	}

	private void configureAuto() {
		swerve.configPathPlanner(
			poseEstimator::getEstimatedPose,
			poseEstimator::resetPose,
			PathPlannerUtil.getGuiRobotConfig()
				.orElse(
					new RobotConfig(
						60,
						0.002,
						new ModuleConfig(
							0.0474588,
							swerve.getConstants().velocityAt12VoltsMetersPerSecond(),
							0.96,
							DCMotor.getKrakenX60Foc(1),
							KrakenX60DriveBuilder.GEAR_RATIO,
							KrakenX60DriveBuilder.SLIP_CURRENT,
							1
						),
						swerve.getModules().getModulePositionsFromCenterMeters()
					)
				)
		);
	}

	public void periodic() {
		BusChain.refreshAll();

		swerve.update();
		poseEstimator.updateOdometry(swerve.getAllOdometryData());
		poseEstimator.updateVision(visionSources.getFilteredVisionData());
		headingEstimator.updateGyroAngle(new TimedValue<>(swerve.getGyroAbsoluteYaw(), TimeUtil.getCurrentTimeSeconds()));
		for (TimedValue<Rotation2d> headingData : visionSources.getFilteredRobotHeading()) {
			headingEstimator.updateVisionIfGyroOffsetIsNotCalibrated(
				headingData,
				RobotHeadingEstimatorConstants.DEFAULT_VISION_STANDARD_DEVIATION,
				RobotHeadingEstimatorConstants.MAXIMUM_STANDARD_DEVIATION_TOLERANCE
			);
		}
		headingEstimator.log();

		Logger.recordOutput("object exist", object.isPresent());
		if (object.isPresent()) {
			Logger.recordOutput("object", object.get());
		} else {
			Logger.recordOutput("object", new Translation2d(-1, -1));
		}


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

	public Optional<Translation2d> getObject() {
		return object;
	}

	public IPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

}
