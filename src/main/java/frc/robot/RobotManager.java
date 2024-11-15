// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.rev.request.SparkMaxRequest;
import frc.robot.hardware.rev.request.SparkMaxRequestBuilder;
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

		SparkMaxRequest<Rotation2d> request1 = SparkMaxRequestBuilder.build(Rotation2d.fromRotations(2), CANSparkBase.ControlType.kVelocity, 0);
		SparkMaxRequest<Rotation2d> request2 = SparkMaxRequestBuilder.build(Rotation2d.fromRotations(3), CANSparkBase.ControlType.kVelocity, 0, RobotManager::test);
		Logger.recordOutput("req1", request1.getSetPoint().getRotations());
		Logger.recordOutput("req2", request2.getSetPoint().getRotations());
		request1.withSetPoint(Rotation2d.fromRotations(10));
		request2.withSetPoint(Rotation2d.fromRotations(5));
		Logger.recordOutput("req1with", request1.getSetPoint().getRotations());
		Logger.recordOutput("req2with", request2.getSetPoint().getRotations());

		this.robot = new Robot();
	}

	public static double test(Rotation2d rotation2d){
		return rotation2d.getRotations();
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
