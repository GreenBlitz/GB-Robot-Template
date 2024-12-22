// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motor.TalonFXMotor;
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

	private final Robot robot;
	private Command autonomousCommand;
	private int roborioCycles;

	private final TalonFXMotor talonFXMotor;

	public RobotManager() {
		LoggerFactory.initializeLogger();
		PathPlannerUtils.startPathfinder();
		BatteryUtils.scheduleLimiter();
		this.roborioCycles = 0;

		this.talonFXMotor = new TalonFXMotor("Test", new Phoenix6DeviceID(1), new SysIdRoutine.Config(), new SimpleMotorSimulation(
			new DCMotorSim(
				LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(1), 0.001, 1),
				DCMotor.getKrakenX60Foc(1)
			)
		));

		TalonFXConfiguration driveConfig = new TalonFXConfiguration();

		driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		driveConfig.Feedback.SensorToMechanismRatio = 1;

		driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60;
		driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -60;
		driveConfig.CurrentLimits.StatorCurrentLimit = 60 ;
		driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

		driveConfig.Slot0.kS = 0.21549;
		driveConfig.Slot0.kV = 0.72124;
		driveConfig.Slot0.kA = 0.11218;
		driveConfig.Slot0.kP = 1.5;
		driveConfig.Slot0.kI = 0;
		driveConfig.Slot0.kD = 0;

		talonFXMotor.applyConfiguration(driveConfig);

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
		this.autonomousCommand = robot.getAutonomousCommand();

		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	@Override
	public void teleopInit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		TalonFXConfiguration steerConfig = new TalonFXConfiguration();

		steerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		steerConfig.CurrentLimits.StatorCurrentLimit = 30;
		steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

		steerConfig.Feedback.RotorToSensorRatio = 3;
		steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

		steerConfig.Slot0.kS = 0.19648;
		steerConfig.Slot0.kV = 2.5763;
		steerConfig.Slot0.kA = 0.50361;
		steerConfig.Slot0.kP = 88;
		steerConfig.Slot0.kI = 0;
		steerConfig.Slot0.kD = 1.5;
		steerConfig.ClosedLoopGeneral.ContinuousWrap = true;
		
		talonFXMotor.applyConfiguration(steerConfig);
	}

	@Override
	public void robotPeriodic() {
		updateTimeRelatedData(); // Better to be first
		CommandScheduler.getInstance().run();
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		AlertManager.reportAlerts();
	}

	private void updateTimeRelatedData() {
		roborioCycles++;
		Logger.recordOutput("RoborioCycles", roborioCycles);
		TimeUtils.updateCycleTime(roborioCycles);
	}

}
