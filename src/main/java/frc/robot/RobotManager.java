// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.motor.EncoderedMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;
import frc.robot.hardware.request.phoenix6.Phoenix6AngleRequest;
import frc.robot.hardware.request.phoenix6.Phoenix6DoubleRequest;
import frc.robot.hardware.signal.phoenix.Phoenix6AngleSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.robot.simulation.SimulationManager;
import frc.utils.AngleUnit;
import frc.utils.DriverStationUtils;
import frc.utils.alerts.AlertManager;
import frc.utils.battery.BatteryUtils;
import frc.utils.brakestate.BrakeStateManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.utils.cycletime.CycleTimeUtils;
import frc.utils.joysticks.JoystickPorts;
import frc.utils.joysticks.SmartJoystick;
import frc.utils.logger.LoggerFactory;
import org.littletonrobotics.junction.LoggedRobot;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the
 * TimedRobot documentation. If you change the name of this class or the package after creating this project, you must also update
 * the build.gradle file in the project.
 */
public class RobotManager extends LoggedRobot {

	private Command autonomousCommand;

	// private Robot robot;
	public EncoderedMotor falcon;
	public Phoenix6AngleSignal position;
	private static final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(JoystickPorts.MAIN);

	@Override
	public void robotInit() {
		LoggerFactory.initializeLogger();
		BatteryUtils.scheduleLimiter();

		TalonFXWrapper mo = new TalonFXWrapper(1);
		falcon = new TalonFXMotor("test motor", mo, new SysIdRoutine.Config());
		position = Phoenix6SignalBuilder.generatePhoenix6Signal(mo.getPosition(), 20, AngleUnit.ROTATIONS);


		MAIN_JOYSTICK.A.onTrue(new InstantCommand(() -> falcon.resetPosition(Rotation2d.fromDegrees(90))));
		MAIN_JOYSTICK.B
			.onTrue(new InstantCommand(() -> falcon.applyAngleRequest(new Phoenix6AngleRequest(new PositionVoltage(3)))));
		MAIN_JOYSTICK.X
			.onTrue(new InstantCommand(() -> falcon.applyAngleRequest(new Phoenix6AngleRequest(new PositionVoltage(10)))));
		MAIN_JOYSTICK.Y
			.onTrue(new InstantCommand(() -> falcon.applyDoubleRequest(new Phoenix6DoubleRequest(new VoltageOut(10)))));

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
//		autonomousCommand = robot.getAutonomousCommand();
//
//		if (autonomousCommand != null) {
//			autonomousCommand.schedule();
//		}
	}

	@Override
	public void teleopInit() {
//		if (autonomousCommand != null) {
//			autonomousCommand.cancel();
//		}
	}

	@Override
	public void robotPeriodic() {
		CycleTimeUtils.updateCycleTime(); // Better to be first
		CommandScheduler.getInstance().run();
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		AlertManager.reportAlerts();

		falcon.updateSignals(position);
	}

	@Override
	public void simulationPeriodic() {
		SimulationManager.updateRegisteredSimulations();
	}

}
