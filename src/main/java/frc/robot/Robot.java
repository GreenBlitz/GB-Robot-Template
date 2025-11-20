// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.subsystems.constants.intakeRollers.IntakeRollerConstants;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.robot.subsystems.flywheel.KrakenX60FlyWheelBuilder;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.SparkMaxRollerBuilder;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType(false);;
	private final FlyWheel flyWheel;
	private final Roller intakeRoller;

	public Robot() {
		BatteryUtil.scheduleLimiter();
		this.flyWheel = KrakenX60FlyWheelBuilder.build("Subsystems/FlyWheel", IDs.TalonFXIDs.FLYWHEEL);
		this.intakeRoller = createIntakeRollers();
	}

	public void periodic() {
		BusChain.refreshAll();

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public Roller createIntakeRollers() {
		return SparkMaxRollerBuilder.build(
			"Subsystems/IntakeRollers",
			IDs.SparkMAXIDs.INTAKE_ROLLERS,
			IntakeRollerConstants.GEAR_RATIO,
			IntakeRollerConstants.CURRENT_LIMIT,
			IntakeRollerConstants.MOMENT_OF_INERTIA
		);
	}

	public FlyWheel getFlyWheel() {
		return flyWheel;
	}

	public Roller getIntakeRoller() {
		return intakeRoller;
	}

	public PathPlannerAutoWrapper getAutonomousCommand() {
		return new PathPlannerAutoWrapper();
	}

}
