// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.IDs;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelComponents;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.flywheel.factory.FlywheelFactory;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final Flywheel flywheel;

	public Robot() {
		FlywheelComponents topFlywheelComponents = FlywheelFactory
			.create(FlywheelConstants.LOG_PATH + "TopMotor", true, IDs.CANSparkMaxIDs.TOP_FLYWHEEL_MOTOR);
		FlywheelComponents bottomFlywheelComponents = FlywheelFactory
			.create(FlywheelConstants.LOG_PATH + "Flywheel/BottomMotor", false, IDs.CANSparkMaxIDs.BOTTOM_FLYWHEEL_MOTOR);
		this.flywheel = new Flywheel(topFlywheelComponents, bottomFlywheelComponents, FlywheelConstants.LOG_PATH);

		configureBindings();
	}

	private void configureBindings() {
		JoysticksBindings.configureBindings(this);
	}


	public Command getAutonomousCommand() {
		return new InstantCommand();
	}

}
