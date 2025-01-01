// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.RobotManager;
import frc.constants.VisionConstants;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.vision.multivisionsources.MultiAprilTagVisionSources;
import frc.utils.battery.BatteryUtils;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();
	private final MultiAprilTagVisionSources aprilTagVisionSources;

	public Robot() {
		BatteryUtils.scheduleLimiter();
		// waiting for swerve and poseestimator to be merged in order to have botpose2 working
		this.aprilTagVisionSources = new MultiAprilTagVisionSources(
			VisionConstants.MULTI_VISION_SOURCES_LOGPATH,
			() -> Rotation2d.fromDegrees(0), // swerve::getGyroAbsoluteYaw,
			() -> Rotation2d.fromDegrees(0), // () -> poseEstimator.getEstimatedPose().getRotation(),
			VisionConstants.DEFAULT_VISION_SOURCES
		);
	}

	public void periodic() {
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		aprilTagVisionSources.log();
		CommandScheduler.getInstance().run();
	}

	public Command getAutonomousCommand() {
		return new InstantCommand();
	}

	public MultiAprilTagVisionSources getAprilTagVisionSources() {
		return aprilTagVisionSources;
	}

}
