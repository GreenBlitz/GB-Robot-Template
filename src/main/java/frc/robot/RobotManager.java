// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.hardware.gyro.phoenix6.Pigeon2Gyro;
import frc.robot.hardware.gyro.phoenix6.Pigeon2Wrapper;
import frc.robot.hardware.signal.AngleSignal;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6AngleSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.robot.simulation.SimulationManager;
import frc.utils.AngleUnit;
import frc.utils.alerts.AlertManager;
import frc.utils.DriverStationUtils;
import frc.utils.battery.BatteryUtils;
import frc.utils.ctre.BusChain;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.cycletime.CycleTimeUtils;
import frc.utils.logger.LoggerFactory;
import org.littletonrobotics.junction.LoggedRobot;
import frc.utils.brakestate.BrakeStateManager;
import org.littletonrobotics.junction.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the
 * TimedRobot documentation. If you change the name of this class or the package after creating this project, you must also update
 * the build.gradle file in the project.
 */
public class RobotManager extends LoggedRobot {

	private Command autonomousCommand;

	private Robot robot;
	private Pigeon2Gyro gyro;
    private Phoenix6AngleSignal yaw,pitch,roll;

	@Override
	public void robotInit() {
		LoggerFactory.initializeLogger();
		BatteryUtils.scheduleLimiter();

        Pigeon2Wrapper wrapper = new Pigeon2Wrapper(new CTREDeviceID(0, BusChain.ROBORIO));
		gyro = new Pigeon2Gyro("gyro",wrapper);
		this.robot = new Robot();
		yaw = Phoenix6SignalBuilder.generatePhoenix6Signal(wrapper.getYaw(), 50,AngleUnit.DEGREES);
		pitch = Phoenix6SignalBuilder.generatePhoenix6Signal(wrapper.getPitch(), 50,AngleUnit.DEGREES);
		roll = Phoenix6SignalBuilder.generatePhoenix6Signal(wrapper.getRoll(), 50,AngleUnit.DEGREES);
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
		CommandScheduler.getInstance().run();
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		AlertManager.reportAlerts();

		gyro.updateSignals(yaw,pitch,roll);
		Logger.recordOutput("yaw",yaw.getLatestValue());
		Logger.recordOutput("pitch",pitch.getLatestValue());
		Logger.recordOutput("roll",roll.getLatestValue());
	}

	@Override
	public void simulationPeriodic() {
		SimulationManager.updateRegisteredSimulations();
	}

}
