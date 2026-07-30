// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.revrobotics.util.StatusLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.constants.field.Field;
import frc.robot.Robot;
import frc.utils.GamePeriodUtils;
import frc.utils.HubUtil;
import frc.utils.alerts.Alert;
import frc.utils.brakestate.BrakeMode;
import frc.utils.driverstation.DriverStationUtil;
import frc.utils.alerts.AlertManager;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.brakestate.BrakeStateManager;
import frc.utils.logger.LoggerFactory;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.stream.Collectors;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update the build.gradle file in
 * the project.
 */
public class RobotManager extends LoggedRobot {

	private final Robot robot;
	private PathPlannerAutoWrapper autonomousCommand;
	private int roborioCycles;
	private static double teleopStartTimeSeconds = -1;
	private String alertsMessage;
	private final Field2d field2d;

	public RobotManager() {
		StatusLogger.disableAutoLogging();
		if (Robot.ROBOT_TYPE.isReplay()) {
			setUseTiming(false);
		}
		DriverStation.silenceJoystickConnectionWarning(true);
		LoggerFactory.initializeLogger();
		PathPlannerUtil.startPathfinder();
		PathPlannerUtil.setupPathPlannerLogging();

		this.roborioCycles = 0;
		this.robot = new Robot();

		JoysticksBindings.configureBindings(robot);

		Threads.setCurrentThreadPriority(true, 10);

		field2d = new Field2d();
		SmartDashboard.putData(field2d);

		robot.getAutonomousChooser().getChooser().onChange((autonomousCommand) -> {
			this.autonomousCommand = autonomousCommand.get();
			field2d.getObject("path").setPoses(robot.getAutonomousChooser().getChosenValue().getPath(!Field.isFieldConventionAlliance()));
		});

		robot.getReturnToMiddleChooser().onChange((val) -> {
			this.autonomousCommand = robot.getAutonomousChooser().getChosenValue();
			field2d.getObject("path").setPoses(robot.getAutonomousChooser().getChosenValue().getPath(!Field.isFieldConventionAlliance()));
		});

		robot.getLimelights().forEach(limelight -> limelight.setThrottleState(!DriverStationUtil.isMatch()));

		alertsMessage = "Alerts: None";
		Logger.recordOutput("AlertsMessage", alertsMessage);
		logDriverAlerts();
	}

	@Override
	public void disabledExit() {
		if (!DriverStationUtil.isMatch()) {
			BrakeStateManager.setBrakeMode(BrakeMode.BRAKE);
		}
		robot.getLimelights().forEach(limelight -> limelight.setThrottleState(false));
	}

	@Override
	public void disabledInit() {
		if (!DriverStationUtil.isMatch()) {
			BrakeStateManager.setBrakeMode(BrakeMode.COAST);
		}
	}

	@Override
	public void autonomousInit() {
		robot.getSwerve().setIsRunningIndependently(true);
		TimeUtil.setAutonomousStartTimeSeconds(TimeUtil.getCurrentTimeSeconds());

		if (autonomousCommand == null) {
			this.autonomousCommand = robot.getAutonomousChooser().getChosenValue();
		}
		CommandScheduler.getInstance().schedule(autonomousCommand);
	}

	@Override
	public void autonomousExit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		if (!DriverStationUtil.isMatch()) {
			robot.getLimelights().forEach(limelight -> limelight.setThrottleState(true));
		}
		robot.getSwerve().setIsRunningIndependently(false);
		Logger.recordOutput("averagePeriodPBS/Autonomous", robot.getAverageBPSForLastXSeconds(GamePeriodUtils.AUTONOMOUS_DURATION_SECONDS));
	}

	@Override
	public void teleopInit() {
		teleopStartTimeSeconds = TimeUtil.getCurrentTimeSeconds();
	}

	@Override
	public void teleopExit() {
		robot.getLimelights().forEach(limelight -> limelight.captureGivenTime(GamePeriodUtils.COMPLETE_GAME_TIME_SECONDS));
		robot.getLimelights().forEach(limelight -> limelight.setThrottleState(true));
	}

	public static double getTeleopStartTimeSeconds() {
		return teleopStartTimeSeconds;
	}

	@Override
	public void simulationPeriodic() {
		robot.getSimulationManager().logPoses();
	}

	@Override
	public void robotPeriodic() {
		updateTimeRelatedData(); // Better to be first
		JoysticksBindings.updateChassisDriverInputs();
		HubUtil.refreshAlliances();
		robot.periodic();
		AlertManager.reportAlerts();
		logElasticRelatedInfo();
	}

	private void updateTimeRelatedData() {
		roborioCycles++;
		Logger.recordOutput("RoborioCycles", roborioCycles);
		TimeUtil.updateCycleTime(roborioCycles);
	}

	private void logElasticRelatedInfo() {
		Logger.recordOutput("isAutoWinner", HubUtil.isRobotAllianceAutoWinnerForLog());
		Logger.recordOutput("TimeUntilNextShift", HubUtil.timeUntilCurrentShiftEndsSeconds(TimeUtil.getTimeSinceTeleopInitSeconds()));
		Logger.recordOutput("IsHubActive", HubUtil.isOurHubActive(TimeUtil.getTimeSinceTeleopInitSeconds()));
		Logger.recordOutput("TimeLeftForGame", GamePeriodUtils.getTimeUntilGameEnds());
		Logger.recordOutput("CurrentGamePeriod", GamePeriodUtils.getCurrentGamePeriod());

		logDriverAlerts();

		field2d.setRobotPose(robot.getPoseEstimator().getEstimatedPose());
	}

	private void logDriverAlerts() {
		ArrayList<Alert> alerts = AlertManager.getReportedAlerts();

		String newAlertsMessage = alerts.stream().filter(Alert::isDriverRelevant).map(Alert::getName).collect(Collectors.joining(", "));

		boolean areAlertsOk = newAlertsMessage.isEmpty();

		Logger.recordOutput("AreAlertsOK", areAlertsOk);

		newAlertsMessage = "Alerts: " + (areAlertsOk ? "None" : newAlertsMessage);

		if (!newAlertsMessage.equals(alertsMessage)) {
			alertsMessage = newAlertsMessage;
			Logger.recordOutput("AlertsMessage", alertsMessage);
		}
	}

}
