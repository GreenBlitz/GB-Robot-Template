// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.robot.subsystems.flywheel.FlyWheelBuilder;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();
	private final FlyWheel flyWheel;

	public Robot() {
		BatteryUtil.scheduleLimiter();
		TalonFXFollowerConfig followerConfig = new TalonFXFollowerConfig();
		followerConfig.followerIDs = new TalonFXFollowerConfig.TalonFXFollowerID[] {
			new TalonFXFollowerConfig.TalonFXFollowerID("follower1", new Phoenix6DeviceID(2, BusChain.ROBORIO), false)};

		this.flyWheel = FlyWheelBuilder.generate("flyWheel", new Phoenix6DeviceID(1, BusChain.ROBORIO), followerConfig);
	}

	public void periodic() {
		BusChain.refreshAll();

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public FlyWheel getFlyWheel() {
		return flyWheel;
	}

	public PathPlannerAutoWrapper getAutonomousCommand() {
		return new PathPlannerAutoWrapper();
	}

}
