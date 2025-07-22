// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

<<<<<<< HEAD
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.led.LEDConstants;
import frc.robot.led.LEDState;
import frc.robot.subsystems.climb.lifter.LifterConstants;
=======
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Threads;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousConstants;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.alerts.AlertManager;
>>>>>>> template/master
import frc.utils.DriverStationUtil;
import frc.utils.alerts.AlertManager;
import frc.utils.auto.PathPlannerUtil;
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
<<<<<<< HEAD
	private Command auto;
=======
	private PathPlannerAutoWrapper autonomousCommand;
>>>>>>> template/master
	private int roborioCycles;

	public RobotManager() {
		Threads.setCurrentThreadPriority(true, 10);

		LoggerFactory.initializeLogger();
		DriverStation.silenceJoystickConnectionWarning(true);
		PathPlannerUtil.startPathfinder();
		PathPlannerUtil.setupPathPlannerLogging();

		this.roborioCycles = 0;
		this.robot = new Robot();

		createAutoReadyForConstructionChooser();
		JoysticksBindings.configureBindings(robot);

		initializeLEDTriggers();

		Threads.setCurrentThreadPriority(true, 10);
	}

	private void initializeLEDTriggers() {
		Trigger noteIn = new Trigger(() -> robot.getRobotCommander().getSuperstructure().isCoralIn());
		noteIn.onTrue(
			robot.getRobotCommander()
				.getLedStateHandler()
				.setState(LEDState.HAS_CORAL)
				.withTimeout(LEDConstants.CORAL_IN_BLINK_TIME_SECONDS)
				.onlyIf(robot.getRobotCommander().getSuperstructure()::isCoralIn)
				.ignoringDisable(true)
		);

		Trigger climbSwitchPressed = new Trigger(() -> robot.getSolenoid().isAtLimitSwitch());
		climbSwitchPressed.onTrue(
			robot.getRobotCommander()
				.getLedStateHandler()
				.setState(LEDState.TOUCHING_LIMIT_SWITCH)
				.withTimeout(LEDConstants.LIMIT_SWITCH_BLINK_TIME_SECONDS)
				.onlyIf(() -> robot.getSolenoid().isAtLimitSwitch())
				.ignoringDisable(true)
		);
	}

	@Override
	public void disabledInit() {
		if (!DriverStationUtil.isMatch()) {
			BrakeStateManager.coast();
		}

		robot.getSwerve().getCommandsBuilder().resetTargetSpeeds().ignoringDisable(true).schedule();
		robot.getRobotCommander().getLedStateHandler().setState(LEDState.DISABLE).ignoringDisable(true).schedule();
	}

	@Override
	public void disabledExit() {
		robot.getRobotCommander().getLedStateHandler().setState(LEDState.IDLE).ignoringDisable(true).schedule();
		if (robot.getLifter().getPosition().getDegrees() < LifterConstants.MINIMUM_ACHIEVABLE_POSITION.getDegrees()) {
			robot.getLifter().resetPosition(LifterConstants.MINIMUM_ACHIEVABLE_POSITION);
		}
	}

	@Override
	public void autonomousInit() {
<<<<<<< HEAD
		robot.getRobotCommander().removeDefaultCommand();

		if (auto == null) {
			this.auto = robot.getAuto();
		}
		auto.schedule();
	}

	@Override
	public void autonomousExit() {
		if (auto != null) {
			auto.cancel();
=======
		if (autonomousCommand == null) {
			this.autonomousCommand = robot.getAutonomousCommand();
>>>>>>> template/master
		}
		autonomousCommand.schedule();
	}

	@Override
<<<<<<< HEAD
	public void teleopInit() {
		robot.getRobotCommander().initializeDefaultCommand();
=======
	public void autonomousExit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
>>>>>>> template/master
	}

	@Override
	public void robotPeriodic() {
		updateTimeRelatedData(); // Better to be first
<<<<<<< HEAD
		JoysticksBindings.setDriversInputsToSwerve(robot.getSwerve());
=======
		JoysticksBindings.updateChassisDriverInputs();
>>>>>>> template/master
		robot.periodic();
		AlertManager.reportAlerts();
	}

	private void createAutoReadyForConstructionChooser() {
		SendableChooser<Boolean> autoReadyForConstructionSendableChooser = new SendableChooser<>();
		autoReadyForConstructionSendableChooser.setDefaultOption("false", false);
		autoReadyForConstructionSendableChooser.addOption("true", true);
		autoReadyForConstructionSendableChooser.onChange(isReady -> {
			if (isReady) {
<<<<<<< HEAD
				auto = robot.getAuto();
				BrakeStateManager.brake();
			} else {
				BrakeStateManager.coast();
=======
				this.autonomousCommand = robot.getAutonomousCommand();
>>>>>>> template/master
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
