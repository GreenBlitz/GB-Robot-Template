// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelConstants;
import frc.robot.subsystems.funnel.factory.FunnelFactory;
import frc.robot.superstructure.Superstructure;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final Funnel funnel;

	private final Superstructure superstructure;

	public Robot() {
<<<<<<< HEAD
		FlywheelComponents topFlywheelComponents = FlywheelFactory
			.create(FlywheelConstants.LOG_PATH + "TopMotor/", FlywheelConstants.IS_TOP_MOTOR_INVERTED, IDs.CANSparkMaxIDs.TOP_FLYWHEEL);
		FlywheelComponents bottomFlywheelComponents = FlywheelFactory
			.create(FlywheelConstants.LOG_PATH + "BottomMotor/", FlywheelConstants.IS_BOTTOM_MOTOR_INVERTED, IDs.CANSparkMaxIDs.BOTTOM_FLYWHEEL);
		this.flywheel = new Flywheel(topFlywheelComponents, bottomFlywheelComponents, FlywheelConstants.LOG_PATH);
=======

		this.funnel = new Funnel(FunnelFactory.create(FunnelConstants.LOG_PATH));
		this.superstructure = new Superstructure(this);
>>>>>>> subsystems-dev

		configureBindings();
	}

	private void configureBindings() {
		JoysticksBindings.configureBindings(this);
	}

	public Command getAutonomousCommand() {
		return new InstantCommand();
	}


	public Funnel getFunnel() {
		return funnel;
	}

	public Superstructure getSuperstructure() {
		return superstructure;
	}

}
