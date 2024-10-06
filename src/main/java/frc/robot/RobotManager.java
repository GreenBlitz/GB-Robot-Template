// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.simulation.SimulationManager;
import frc.robot.subsystems.intake.IntakeRollerCommandsBuilder;
import frc.utils.alerts.AlertManager;
import frc.utils.DriverStationUtils;
import frc.utils.battery.BatteryUtils;
import frc.utils.cycletime.CycleTimeUtils;
import frc.utils.logger.LoggerFactory;
import frc.utils.pathplannerutils.PathPlannerUtils;
import org.littletonrobotics.junction.LoggedRobot;
import frc.utils.brakestate.BrakeStateManager;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update the build.gradle file in
 * the project.
 */
public class RobotManager extends LoggedRobot {

	private Command autonomousCommand;
	private Robot robot;

	private IntakeRollerCommandsBuilder intakeRollerCommandsBuilder;

	@Override
	public void robotInit() {
		LoggerFactory.initializeLogger();
		PathPlannerUtils.startPathPlanner();
		BatteryUtils.scheduleLimiter();

		this.robot = new Robot();
	}

	@Override
	public void disabledInit() {
		if (!DriverStationUtils.isMatch()) {
			BrakeStateManager.coast();
		}
	}

	@Override
	public void disabledExit() {
		if (!DriverStationUtils.isMatch()) {
			BrakeStateManager.brake();
		}
	}

	@Override
	public void autonomousInit() {
		autonomousCommand = robot.getAutonomousCommand();

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void robotPeriodic() {
		CycleTimeUtils.updateCycleTime(); // Better to be first
		robot.getSwerve().updateState();
		CommandScheduler.getInstance().run();
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		AlertManager.reportAlerts();
		robot.getSuperstructure().logStatus();
	}

	@Override
	public void simulationPeriodic() {
		SimulationManager.updateRegisteredSimulations();
	}

}
