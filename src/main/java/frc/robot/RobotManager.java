// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.hardware.ConnectedInputAutoLogged;
import frc.robot.hardware.encoder.CANCoderEncoder;
import frc.robot.hardware.signal.phoenix.Phoenix6AngleSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6DoubleSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6LatencySignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.robot.simulation.SimulationManager;
import frc.utils.AngleUnit;
import frc.utils.alerts.AlertManager;
import frc.utils.DriverStationUtils;
import frc.utils.battery.BatteryUtils;
import frc.utils.ctre.BusChain;
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

	private CANcoder caNcoder;
	private CANCoderEncoder canCoderEncoder;
	private ConnectedInputAutoLogged connectedInputAutoLogged;
	private Phoenix6AngleSignal angleSignal;
	private Phoenix6DoubleSignal doubleSignal;

//	private Robot robot;

	@Override
	public void robotInit() {
		LoggerFactory.initializeLogger();
		BatteryUtils.scheduleLimiter();

		caNcoder = new CANcoder(1);
		canCoderEncoder  = new CANCoderEncoder(caNcoder);
		connectedInputAutoLogged = new ConnectedInputAutoLogged();
		angleSignal = Phoenix6SignalBuilder.generatePhoenix6Signal(caNcoder.getPosition(), 50, AngleUnit.ROTATIONS);
		doubleSignal = Phoenix6SignalBuilder.generatePhoenix6Signal(caNcoder.getPosition(), 50);



//		this.robot = new Robot();
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
//`		autonomousCommand = robot.getAutonomousCommand();
//
//		if (autonomousCommand != null) {
//			autonomousCommand.schedule();
//		}`
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void robotPeriodic() {



		canCoderEncoder.updateInputs(connectedInputAutoLogged);
		canCoderEncoder.updateSignals(angleSignal, doubleSignal);
		Logger.processInputs("angleSignal", angleSignal);
		Logger.processInputs("doubleSignal", doubleSignal);
		Logger.recordOutput("isok", canCoderEncoder.isOK());
		Logger.processInputs("connected", connectedInputAutoLogged);
		Logger.recordOutput("isoriginOk", BaseStatusSignal.isAllGood(caNcoder.getPosition()));

		CycleTimeUtils.updateCycleTime(); // Better to be first
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
