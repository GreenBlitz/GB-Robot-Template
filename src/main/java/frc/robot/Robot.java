// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.rev.motors.BrushlessSparkMAXMotor;
import frc.robot.hardware.rev.motors.SparkMaxDeviceID;
import frc.robot.hardware.rev.motors.SparkMaxWrapper;
import frc.robot.subsystems.motorSubsystem.MotorSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();
	public MotorSubsystem motorSubsystem;

	public Robot() {
		configureBindings();
		this.motorSubsystem = new MotorSubsystem(
			new BrushlessSparkMAXMotor("motor", new SparkMaxWrapper(new SparkMaxDeviceID(12)), new SysIdRoutine.Config()),
			"motor"
		);
	}

	private void configureBindings() {
		JoysticksBindings.configureBindings(this);
	}

	public Command getAutonomousCommand() {
		return new InstantCommand();
	}

}
