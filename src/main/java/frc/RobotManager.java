// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.hardware.mechanisms.MechanismSimulation;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.TalonFXFollowerConfig;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.utils.DriverStationUtil;
import frc.utils.alerts.AlertManager;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.brakestate.BrakeStateManager;
import frc.utils.logger.LoggerFactory;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update the build.gradle file in
 * the project.
 */
public class RobotManager extends LoggedRobot {

	private final Robot robot;
	private PathPlannerAutoWrapper autonomousCommand;
	private int roborioCycles;
	private TalonFXMotor motor;

	public RobotManager() {
		LoggerFactory.initializeLogger();
		PathPlannerUtil.startPathfinder();
		PathPlannerUtil.setupPathPlannerLogging();

		this.roborioCycles = 0;
		this.robot = new Robot();

		TalonFXFollowerConfig followerConfig = new TalonFXFollowerConfig();
		followerConfig.names = new String[]{"67"};
		followerConfig.followerIDs = new Phoenix6DeviceID[]{new Phoenix6DeviceID(1)};
		followerConfig.followerBuses = new BusChain[]{BusChain.ROBORIO};
		followerConfig.mechanismSimulations = new MechanismSimulation[]{
				new SimpleMotorSimulation(
						new DCMotorSim(
								LinearSystemId.createDCMotorSystem(
										DCMotor.getKrakenX60(1),
										0.001,
										1
								),
								DCMotor.getKrakenX60(1)
						)
				)
		};
		followerConfig.followerConfig = new TalonFXConfiguration();
		followerConfig.followerOpposeMain = new boolean[]{false};

		motor = new TalonFXMotor(
				"tester/",
				new Phoenix6DeviceID(2),
				followerConfig,
				new SysIdRoutine.Config(),
				new SimpleMotorSimulation(
						new DCMotorSim(
								LinearSystemId.createDCMotorSystem(
										DCMotor.getKrakenX60(1),
										0.001,
										1
								),
								DCMotor.getKrakenX60(1)
						)
				)
		);

        createAutoReadyForConstructionChooser();
		JoysticksBindings.configureBindings(robot);

		Threads.setCurrentThreadPriority(true, 10);
	}

	@Override
	public void disabledInit() {
		if (!DriverStationUtil.isMatch()) {
			BrakeStateManager.coast();
		}
	}

	@Override
	public void disabledExit() {
		if (!DriverStationUtil.isMatch()) {
			BrakeStateManager.brake();
		}
	}

	@Override
	public void teleopInit() {
		motor.setPower(1);
	}

	@Override
	public void autonomousInit() {
		if (autonomousCommand == null) {
			this.autonomousCommand = robot.getAutonomousCommand();
		}
		autonomousCommand.schedule();
	}

	@Override
	public void autonomousExit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
	}

	@Override
	public void robotPeriodic() {
		updateTimeRelatedData(); // Better to be first
		JoysticksBindings.updateChassisDriverInputs();
		robot.periodic();
		AlertManager.reportAlerts();

		motor.updateSimulation();
		Logger.recordOutput(motor.getLogPath(), motor.getDevice().get());
	}

	private void createAutoReadyForConstructionChooser() {
		SendableChooser<Boolean> autoReadyForConstructionSendableChooser = new SendableChooser<>();
		autoReadyForConstructionSendableChooser.setDefaultOption("false", false);
		autoReadyForConstructionSendableChooser.addOption("true", true);
		autoReadyForConstructionSendableChooser.onChange(isReady -> {
			if (isReady) {
				this.autonomousCommand = robot.getAutonomousCommand();
				BrakeStateManager.brake();
			} else {
				BrakeStateManager.coast();
			}
			Logger.recordOutput(AutonomousConstants.LOG_PATH_PREFIX + "/ReadyToConstruct", isReady);
		});
		SmartDashboard.putData("AutoReadyForConstruction", autoReadyForConstructionSendableChooser);
	}

	private void updateTimeRelatedData() {
		roborioCycles++;
		Logger.recordOutput("RoborioCycles", roborioCycles);
		TimeUtil.updateCycleTime(roborioCycles);
	}

}
