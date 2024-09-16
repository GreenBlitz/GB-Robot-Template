// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.hardware.signal.phoenix.Phoenix6Thread;
import frc.robot.simulation.SimulationManager;
import frc.utils.alerts.AlertManager;
import frc.utils.DriverStationUtils;
import frc.utils.battery.BatteryUtils;
import frc.utils.ctre.BusChain;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.cycletime.CycleTimeUtils;
import frc.utils.devicewrappers.TalonFXWrapper;
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

	private TalonFXWrapper motor = new TalonFXWrapper(new CTREDeviceID(0, BusChain.CANIVORE));
	private TalonFXWrapper motor1 = new TalonFXWrapper(new CTREDeviceID(1, BusChain.CANIVORE));
	private TalonFXWrapper motor2 = new TalonFXWrapper(new CTREDeviceID(2, BusChain.CANIVORE));
	private TalonFXWrapper motor3 = new TalonFXWrapper(new CTREDeviceID(3, BusChain.CANIVORE));
	private TalonFXWrapper motor4 = new TalonFXWrapper(new CTREDeviceID(4, BusChain.CANIVORE));
	private TalonFXWrapper motor5 = new TalonFXWrapper(new CTREDeviceID(5, BusChain.CANIVORE));
	private TalonFXWrapper motor6 = new TalonFXWrapper(new CTREDeviceID(6, BusChain.CANIVORE));
	private TalonFXWrapper motor7 = new TalonFXWrapper(new CTREDeviceID(7, BusChain.CANIVORE));

	@Override
	public void robotInit() {
		LoggerFactory.initializeLogger();
		BatteryUtils.scheduleLimiter();

		Phoenix6Thread.getInstance().registerSignal(motor.getPosition(), motor.getVelocity());
		Phoenix6Thread.getInstance().registerSignal(motor1.getPosition());
		Phoenix6Thread.getInstance().registerSignal(motor2.getPosition());
		Phoenix6Thread.getInstance().registerSignal(motor3.getPosition());
		Phoenix6Thread.getInstance().registerSignal(motor4.getPosition());
		Phoenix6Thread.getInstance().registerSignal(motor5.getPosition());
		Phoenix6Thread.getInstance().registerSignal(motor6.getPosition());
		Phoenix6Thread.getInstance().registerSignal(motor7.getPosition());
		motor.optimizeBusUtilization();
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
		CommandScheduler.getInstance().run();
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		AlertManager.reportAlerts();

		Phoenix6Thread.LOCK.lock();
		Logger.recordOutput("timestamps", Phoenix6Thread.getInstance().getTimestampsQueue().stream().mapToDouble(Double::doubleValue).toArray());
		Phoenix6Thread.getInstance().getTimestampsQueue().clear();
		Phoenix6Thread.LOCK.unlock();
	}

	@Override
	public void simulationPeriodic() {
		SimulationManager.updateRegisteredSimulations();
	}

}
