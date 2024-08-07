// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.bindings.KeyboardBindings;
import frc.utils.RobotTypeUtils;
import frc.robot.bindings.JoysticksBindings;
import frc.utils.controllers.keyboard.KeyboardController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little
 * robot logic should actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead,
 * the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotTypeUtils.RobotType ROBOT_TYPE = RobotTypeUtils.determineRobotType(RobotTypeUtils.RobotType.REAL);

	public Robot() {
		configureBindings();
	}

	private void configureBindings() {
		JoysticksBindings.configureBindings();
		if (KeyboardController.ENABLE_KEYBOARD) {
			KeyboardBindings.configureBindings();
		}
	}

	public Command getAutonomousCommand() {
		return new InstantCommand();
	}

}
