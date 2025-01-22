// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorSimulationHelper;
import frc.robot.subsystems.elevator.factory.ElevatorFactory;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.factory.EndEffectorFactory;
import frc.utils.battery.BatteryUtils;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final Elevator elevator;
	private final EndEffector endEffector;

	public Robot() {
		BatteryUtils.scheduleLimiter();

		this.elevator = ElevatorFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Elevator");
		this.endEffector = EndEffectorFactory.create();
	}

	public void periodic() {
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last

		Logger.recordOutput("pose", new Pose2d());
		Logger.recordOutput("first stafe", ElevatorSimulationHelper.getFirstElevatorStagePose(elevator.getElevatorPositionMeters()));
		Logger.recordOutput("second stafe", ElevatorSimulationHelper.getSecondElevatorStagePose(elevator.getElevatorPositionMeters()));
	}

	public Command getAutonomousCommand() {
		return new InstantCommand();
	}

	public Elevator getElevator() {
		return elevator;
	}

	public EndEffector getEndEffector() {
		return endEffector;
	}

}
