// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.request.cansparkmax.SparkMaxAngleRequest;
import frc.robot.hardware.request.cansparkmax.SparkMaxDoubleRequest;
import frc.robot.hardware.request.phoenix6.Phoenix6AngleRequest;
import frc.robot.hardware.request.phoenix6.Phoenix6DoubleRequest;
import frc.robot.simulation.SimulationManager;
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

	private Command autonomousCommand;

	private Robot robot;

	@Override
	public void robotInit() {
		LoggerFactory.initializeLogger();
		PathPlannerUtils.startPathfinder();
		BatteryUtils.scheduleLimiter();

		SparkMaxDoubleRequest doubleRequest = new SparkMaxDoubleRequest(1, SparkMaxDoubleRequest.SparkDoubleRequestType.VOLTAGE, 0);
		doubleRequest.withSetPoint(5.0);
		Logger.recordOutput("sparkDouble", doubleRequest.getSetPoint());

		SparkMaxAngleRequest angleRequest = new SparkMaxAngleRequest(Rotation2d.fromRotations(0), SparkMaxAngleRequest.SparkAngleRequestType.VELOCITY, 0);
		angleRequest.withSetPoint(Rotation2d.fromRotations(200));
		Logger.recordOutput("sparkAngle", angleRequest.getSetPoint().getRotations());

		Phoenix6AngleRequest angleRequest1 = new Phoenix6AngleRequest(new PositionVoltage(1));
		angleRequest1.withSetPoint(Rotation2d.fromRotations(15));
		Logger.recordOutput("phonixAngle", angleRequest1.getSetPoint().getRotations());

		Phoenix6DoubleRequest doubleRequest1 = new Phoenix6DoubleRequest(new VoltageOut(5));
		doubleRequest1.withSetPoint(10.0);
		Logger.recordOutput("phonixDouble", doubleRequest1.getSetPoint());

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
		TimeUtils.updateCycleTime(); // Better to be first
		CommandScheduler.getInstance().run();
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		AlertManager.reportAlerts();
	}

	@Override
	public void simulationPeriodic() {
		SimulationManager.updateRegisteredSimulations();
	}

}
