// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.hardware.mechanisms.SingleJointedArmSimulation;
import frc.robot.hardware.phoenix6.BusChain;
import frc.utils.auto.PathPlannerUtils;
import frc.utils.alerts.AlertManager;
import frc.utils.DriverStationUtils;
import frc.utils.battery.BatteryUtils;
import frc.utils.time.TimeUtils;
import frc.utils.logger.LoggerFactory;
import org.littletonrobotics.junction.LoggedRobot;
import frc.utils.brakestate.BrakeStateManager;
import org.littletonrobotics.junction.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update the build.gradle file in
 * the project.
 */
public class RobotManager extends LoggedRobot {

	private int roborioCycles;

	private Command autonomousCommand;

	private Robot robot;

	@Override
	public void robotInit() {
		LoggerFactory.initializeLogger();
		PathPlannerUtils.startPathfinder();
		BatteryUtils.scheduleLimiter();
		roborioCycles = 0;

		SingleJointedArmSim armSim = new SingleJointedArmSim(
				DCMotor.getFalcon500(1),
				(28.0 * (60.0 / 16.0)),
				SingleJointedArmSim.estimateMOI(
						0.44,
						0.44
				),
				0.44,
				Rotation2d.fromDegrees(-81).getRadians(),
				Rotation2d.fromDegrees(90).getRadians(),
				false,
				Rotation2d.fromDegrees(0).getRadians()
		);
		SingleJointedArmSimulation simulation = new SingleJointedArmSimulation(
				armSim,
				(28.0 * (60.0 / 16.0))
		);
		simulation.setInputVoltage(BatteryUtils.DEFAULT_VOLTAGE);
		simulation.s

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
		roborioCycles++; // Better to be first
		Logger.recordOutput("RoborioCycles", roborioCycles);
		TimeUtils.updateCycleTime(roborioCycles); // Better to be second
		CommandScheduler.getInstance().run();
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		AlertManager.reportAlerts();
	}

}
