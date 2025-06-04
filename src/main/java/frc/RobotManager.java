// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.hardware.mechanisms.wpilib.SingleJointedArmSimulation;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.led.LEDConstants;
import frc.robot.led.LEDState;
import frc.robot.subsystems.algaeIntake.pivot.PivotConstants;
import frc.robot.subsystems.climb.lifter.LifterConstants;
import frc.utils.alerts.AlertManager;
import frc.utils.auto.PathPlannerUtil;
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
	private Command auto;
	private int roborioCycles;
	TalonFXMotor test = new TalonFXMotor(
		"teest" + "/Motor",
		new Phoenix6DeviceID(17),
		new SysIdRoutine.Config(),
		new SingleJointedArmSimulation(
			new SingleJointedArmSim(
				LinearSystemId.createDCMotorSystem(
					DCMotor.getKrakenX60Foc(1),
					SingleJointedArmSim.estimateMOI(PivotConstants.LENGTH_METERS, PivotConstants.MASS_KG),
					1.0
				),
				DCMotor.getKrakenX60Foc(1),
				1.0,
				PivotConstants.LENGTH_METERS,
				Rotation2d.fromDegrees(5).getRadians(),
				Rotation2d.fromDegrees(200).getRadians(),
				false,
				PivotConstants.STARTING_POSITION.getRadians()
			),
			1.0
		)
	);
//
//
//	IRequest<Rotation2d> positionRequest = Phoenix6RequestBuilder
//			.build(new PositionVoltage(PivotConstants.STARTING_POSITION.getRotations()), 0, true);
//
//	InputSignal<Rotation2d> positionSignal = Phoenix6SignalBuilder
//			.build(test.getDevice().getPosition(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS, BusChain.ROBORIO);
//	InputSignal<Double> voltageSignal = Phoenix6SignalBuilder
//			.build(test.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, BusChain.ROBORIO);


	public RobotManager() {
		LoggerFactory.initializeLogger();
		DriverStation.silenceJoystickConnectionWarning(true);
		PathPlannerUtil.startPathfinder();
		PathPlannerUtil.setupPathPlannerLogging();
//		test.applyConfiguration(new TalonFXConfiguration().withSlot0(new Slot0Configs().withKP(1)));

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
		robot.getRobotCommander().removeDefaultCommand();

		if (auto == null) {
			this.auto = robot.getAuto();
		}
		auto.schedule();
		robot.getPivot().pivot.applyRequest(robot.getPivot().positionRequest.withSetPoint(Rotation2d.fromDegrees(90)));
	}

	@Override
	public void autonomousExit() {
		if (auto != null) {
			auto.cancel();
		}
	}

	@Override
	public void teleopInit() {
		robot.getRobotCommander().initializeDefaultCommand();
//		robot.getPivot().setPower(0.5);
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
