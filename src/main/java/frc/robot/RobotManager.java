// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.motor.Controllable;
import frc.robot.hardware.motor.cansparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.request.cansparkmax.SparkMaxAngleRequest;
import frc.robot.hardware.request.cansparkmax.SparkMaxDoubleRequest;
import frc.robot.hardware.signal.AngleSignal;
import frc.robot.simulation.SimulationManager;
import frc.utils.AngleUnit;
import frc.utils.DriverStationUtils;
import frc.utils.alerts.AlertManager;
import frc.utils.battery.BatteryUtils;
import frc.utils.brakestate.BrakeStateManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.utils.cycletime.CycleTimeUtils;
import frc.utils.logger.LoggerFactory;
import org.littletonrobotics.junction.LoggedRobot;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update the build.gradle file in
 * the project.
 */
public class RobotManager extends LoggedRobot {

	private Command autonomousCommand;

//	private Robot robot;
	private Controllable zbark;
	private AngleSignal position;

	@Override
	public void robotInit() {
		LoggerFactory.initializeLogger();
		BatteryUtils.scheduleLimiter();

		CANSparkMax bldcmotor = new CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless);
		zbark = new BrushlessSparkMAXMotor(bldcmotor,(a,b) -> {return Rotation2d.fromRadians(0)},new SysIdRoutine.Config(), "test motor");

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
		zbark.resetPosition(Rotation2d.fromRotations(10));
	}

	@Override
	public void robotPeriodic() {
		CycleTimeUtils.updateCycleTime(); // Better to be first
		CommandScheduler.getInstance().run();
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		AlertManager.reportAlerts();

		//todo - on run, comment out one of the below and run, and than run the other and comment out the other one
		zbark.applyDoubleRequest(new SparkMaxDoubleRequest(10, SparkMaxDoubleRequest.SparkDoubleRequestType.VOLTAGE,0));
//		zbark.applyAngleRequest(new SparkMaxAngleRequest(Rotation2d.fromRotations(5), SparkMaxAngleRequest.SparkAngleRequestType.POSITION,0));
	}

	@Override
	public void simulationPeriodic() {
		SimulationManager.updateRegisteredSimulations();
	}

}
