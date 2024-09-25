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
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.motor.cansparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.request.cansparkmax.SparkMaxAngleRequest;
import frc.robot.hardware.signal.cansparkmax.SparkMaxAngleSignal;
import frc.robot.simulation.SimulationManager;
import frc.utils.AngleUnit;
import frc.utils.alerts.AlertManager;
import frc.utils.DriverStationUtils;
import frc.utils.battery.BatteryUtils;
import frc.utils.cycletime.CycleTimeUtils;
import frc.utils.logger.LoggerFactory;
import org.littletonrobotics.junction.LoggedRobot;
import frc.utils.brakestate.BrakeStateManager;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the
 * TimedRobot documentation. If you change the name of this class or the package after creating this project, you must also update
 * the build.gradle file in the project.
 */
public class RobotManager extends LoggedRobot {

	private Command autonomousCommand;

	private Robot robot;
	private ControllableMotor sparkmax;
	private SparkMaxAngleSignal position;

	@Override
	public void robotInit() {
		LoggerFactory.initializeLogger();
		BatteryUtils.scheduleLimiter();

		CANSparkMax motor = new CANSparkMax(11, CANSparkLowLevel.MotorType.kBrushless);
		position = new SparkMaxAngleSignal("positoin", () -> motor.getEncoder().getPosition(), AngleUnit.RADIANS);
		sparkmax = new BrushlessSparkMAXMotor(motor,(a,b) -> {return Rotation2d.fromRotations(0);},new SysIdRoutine.Config(), "loggg");

		motor.getPIDController().setP(2);

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
		sparkmax.resetPosition(new Rotation2d());
		sparkmax.setBrake(true);
	}

	@Override
	public void robotPeriodic() {
		CycleTimeUtils.updateCycleTime(); // Better to be first
		CommandScheduler.getInstance().run();
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		AlertManager.reportAlerts();

		sparkmax.applyAngleRequest(new SparkMaxAngleRequest(Rotation2d.fromRotations(10), SparkMaxAngleRequest.SparkAngleRequestType.POSITION,0));
		sparkmax.updateSignals(position);
	}

	@Override
	public void simulationPeriodic() {
		SimulationManager.updateRegisteredSimulations();
	}

}
