// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.led.LEDState;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.alerts.AlertManager;
import frc.utils.DriverStationUtil;
import frc.utils.time.TimeUtil;
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
	private PathPlannerAutoWrapper auto;
	private int roborioCycles;

	public RobotManager() {
		LoggerFactory.initializeLogger();
		PathPlannerUtil.startPathfinder();

		this.roborioCycles = 0;
		this.robot = new Robot();

		createAutoReadyForConstructionChooser();
		JoysticksBindings.configureBindings(robot);
	}

	@Override
	public void disabledInit() {
		if (!DriverStationUtil.isMatch()) {
			BrakeStateManager.coast();
		}
		
		robot.ledStateHandler.setState(LEDState.DISABLE).schedule();
	}

	@Override
	public void disabledExit() {
		BrakeStateManager.brake();
	}

	@Override
	public void autonomousInit() {
		robot.getRobotCommander().removeDefaultCommand();

		if (auto == null) {
			this.auto = robot.getAuto();
		}
		auto.schedule();
	}

	@Override
	public void teleopInit() {
		if (auto != null) {
			auto.cancel();
		}
		robot.getRobotCommander().initializeDefaultCommand();
	}

	@Override
	public void robotPeriodic() {
		updateTimeRelatedData(); // Better to be first
		JoysticksBindings.setDriversInputsToSwerve(robot.getSwerve());
		robot.periodic();
		AlertManager.reportAlerts();
	}

	private void createAutoReadyForConstructionChooser() {
		SendableChooser<Boolean> autoReadyForConstructionSendableChooser = new SendableChooser<>();
		autoReadyForConstructionSendableChooser.setDefaultOption("false", false);
		autoReadyForConstructionSendableChooser.addOption("true", true);
		autoReadyForConstructionSendableChooser.onChange(isReady -> {
			if (isReady) {
				auto = robot.getAuto();
			}
		});
		SmartDashboard.putData("AutoReadyForConstruction", autoReadyForConstructionSendableChooser);
	}

	private void updateTimeRelatedData() {
		roborioCycles++;
		Logger.recordOutput("RoborioCycles", roborioCycles);
		TimeUtil.updateCycleTime(roborioCycles);
	}

}
