// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousConstants;
import frc.utils.DriverStationUtil;
import frc.utils.alerts.AlertManager;
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
	private Command autonomousCommand;
	private int roborioCycles;

	public RobotManager() {
		LoggerFactory.initializeLogger();
//		PathPlannerUtil.startPathfinder();
//		PathPlannerUtil.setupPathPlannerLogging();

		this.roborioCycles = 0;
		this.robot = new Robot();

//		createAutoReadyForConstructionChooser();
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
		switch (Robot.TEAM_NUMBER) {
			case 0 ->
				robot.getMecanumDrive()
					.driveCartesian(
						JoysticksBindings.chassisDriverInputs.xPower,
						JoysticksBindings.chassisDriverInputs.yPower,
						JoysticksBindings.chassisDriverInputs.rotationalPower
					);
			case 1, 2 ->
				robot.getTankDrive()
					.arcadeDrive(JoysticksBindings.chassisDriverInputs.xPower, JoysticksBindings.chassisDriverInputs.rotationalPower);
		}
		AlertManager.reportAlerts();
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
