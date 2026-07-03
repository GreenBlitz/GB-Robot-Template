package frc;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.statemachine.RobotState;
import frc.robot.statemachine.intakestatehandler.IntakeState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.robot.subsystems.swerve.states.DriveSpeed;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.utils.HubUtil;
import frc.utils.battery.BatteryUtil;
import frc.utils.time.TimeUtil;
import frc.utils.utilcommands.ExecuteEndCommand;

public class JoysticksBindings {

	private static final double PRE_SHIFT_END_RUMBLE_TIME = 1.0;
	private static final double PRE_SHIFT_END_RUMBLE_POWER = 0.5;
	private static final double TIME_BEFORE_SHIFT_END_TO_RUMBLE = 5.0;

	private static final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(JoystickPorts.MAIN, true);
	private static final SmartJoystick SECOND_JOYSTICK = new SmartJoystick(JoystickPorts.SECOND);
	private static final SmartJoystick THIRD_JOYSTICK = new SmartJoystick(JoystickPorts.THIRD);
	private static final SmartJoystick FOURTH_JOYSTICK = new SmartJoystick(JoystickPorts.FOURTH);
	private static final SmartJoystick FIFTH_JOYSTICK = new SmartJoystick(JoystickPorts.FIFTH);
	private static final SmartJoystick SIXTH_JOYSTICK = new SmartJoystick(JoystickPorts.SIXTH);

	private static final ChassisPowers chassisDriverInputs = new ChassisPowers();

	private static Command preShiftEndJoystickRumble(SmartJoystick joystick) {
		return new ExecuteEndCommand(
			() -> joystick.setRumble(GenericHID.RumbleType.kBothRumble, PRE_SHIFT_END_RUMBLE_POWER),
			() -> joystick.stopRumble(GenericHID.RumbleType.kBothRumble)
		).withTimeout(PRE_SHIFT_END_RUMBLE_TIME);
	}

	public static void configureBindings(Robot robot) {
		robot.getSwerve().setDriversPowerInputs(chassisDriverInputs);

		mainJoystickButtons(robot);
		secondJoystickButtons(robot);
		thirdJoystickButtons(robot);
		fourthJoystickButtons(robot);
		fifthJoystickButtons(robot);
		sixthJoystickButtons(robot);
	}

	public static void updateChassisDriverInputs() {
		if (MAIN_JOYSTICK.isConnected()) {
			chassisDriverInputs.xPower = MAIN_JOYSTICK.getAxisValue(Axis.LEFT_Y);
			chassisDriverInputs.yPower = MAIN_JOYSTICK.getAxisValue(Axis.LEFT_X);
			chassisDriverInputs.rotationalPower = MAIN_JOYSTICK.getAxisValue(Axis.RIGHT_X);
		} else if (THIRD_JOYSTICK.isConnected()) {
			chassisDriverInputs.xPower = THIRD_JOYSTICK.getAxisValue(Axis.LEFT_Y);
			chassisDriverInputs.yPower = THIRD_JOYSTICK.getAxisValue(Axis.LEFT_X);
			chassisDriverInputs.rotationalPower = THIRD_JOYSTICK.getAxisValue(Axis.RIGHT_X);
		} else {
			chassisDriverInputs.xPower = 0;
			chassisDriverInputs.yPower = 0;
			chassisDriverInputs.rotationalPower = 0;
		}
	}

	public static SwerveState getScoringState(SmartJoystick joystick) {
		return RobotState.SCORE.getSwerveState().withDriveSpeed(joystick.R1.getAsBoolean() ? DriveSpeed.BOOST_SHOOT : DriveSpeed.SHOOT);
	}

	public static SwerveState getPassState(SmartJoystick joystick) {
		return RobotState.PASS.getSwerveState()
			.withDriveSpeed(joystick.getAxisAsButton(Axis.RIGHT_TRIGGER).getAsBoolean() ? DriveSpeed.BOOST_PASS : DriveSpeed.PASS);
	}

	private static Command driveActionChooser(Robot robot) {
		return new InstantCommand(() -> {
			Command intakeCommand = Commands.none();
			if (robot.getRobotCommander().getCurrentState() == RobotState.OUTTAKE) {
				intakeCommand = robot.getRobotCommander().getIntakeStateHandler().intake();
			}
			CommandScheduler.getInstance()
				.schedule(new ParallelCommandGroup(robot.getRobotCommander().driveWith(RobotState.NEUTRAL), intakeCommand));
		});
	}

	private static void mainJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = MAIN_JOYSTICK;

		Trigger preShiftEndJoystickRumble = new Trigger(
			() -> HubUtil.timeUntilCurrentShiftEndsSeconds(TimeUtil.getTimeSinceTeleopInitSeconds()) <= TIME_BEFORE_SHIFT_END_TO_RUMBLE
		).onTrue(preShiftEndJoystickRumble(usedJoystick));

		usedJoystick.A.onTrue(driveActionChooser(robot));

		// Shoot & Pass...
		usedJoystick.R1.onTrue(
			robot.getRobotCommander()
				.driveWithChangingState(RobotState.PRE_SCORE, robot.getRobotCommander().scoreSequence(), () -> getScoringState(usedJoystick))
		);
		usedJoystick.getAxisAsButton(Axis.RIGHT_TRIGGER)
			.onTrue(
				robot.getRobotCommander()
					.driveWithChangingState(RobotState.PRE_PASS, robot.getRobotCommander().passSequence(), () -> getPassState(usedJoystick))
			);
		usedJoystick.START.onTrue(new InstantCommand(() -> robot.getSwerve().getStateHandler().enableAimAssist(true)));
		usedJoystick.BACK.onTrue(new InstantCommand(() -> robot.getSwerve().getStateHandler().enableAimAssist(false)));

		// Intake binds...
		robot.getRobotCommander().getIntakeStateHandler().setIntakeButtonsSuppliers(usedJoystick.getAxisAsButton(Axis.LEFT_TRIGGER));
		usedJoystick.getAxisAsButton(Axis.LEFT_TRIGGER).onTrue(robot.getRobotCommander().getIntakeStateHandler().setState(IntakeState.INTAKE));
		usedJoystick.L1.onTrue(robot.getRobotCommander().getIntakeStateHandler().setState(IntakeState.CLOSED));
		usedJoystick.POV_RIGHT.onTrue(
			robot.getRobotCommander()
				.getIntakeStateHandler()
				.setState(IntakeState.SLOW_CLOSE)
				.andThen(robot.getRobotCommander().getIntakeStateHandler().setState(IntakeState.INTAKE))
		);
		usedJoystick.B.onTrue(robot.getRobotCommander().driveWith(RobotState.OUTTAKE));
		usedJoystick.Y.onTrue(robot.getRobotCommander().getIntakeStateHandler().setState(IntakeState.OUTTAKE));
		usedJoystick.POV_DOWN.onTrue(robot.getRobotCommander().driveWith(RobotState.CONVEYOR_OUTTAKE));
		usedJoystick.X.onTrue(new InstantCommand(() -> robot.getRobotCommander().setIsInDefenceMode(true)));
		usedJoystick.X.onFalse(new InstantCommand(() -> robot.getRobotCommander().setIsInDefenceMode(false)));
	}

	private static void secondJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SECOND_JOYSTICK;
		// bindings...
	}

	private static void thirdJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = THIRD_JOYSTICK;
		// bindings...
		applyInterpolationCalibrationBindings(usedJoystick, robot);
	}

	private static void fourthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = FOURTH_JOYSTICK;
		// bindings...
	}

	private static void fifthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = FIFTH_JOYSTICK;
		// bindings...
	}

	private static void sixthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SIXTH_JOYSTICK;
		// bindings...
	}

	private static void applyInterpolationCalibrationBindings(SmartJoystick joystick, Robot robot) {
		joystick.A.onTrue(robot.getRobotCommander().driveWith(RobotState.NEUTRAL));
		joystick.Y.onTrue(robot.getRobotCommander().scoreSequence());
		joystick.POV_LEFT.onTrue(robot.getRobotCommander().calibrationScoreSequence());
		joystick.POV_RIGHT.whileTrue(robot.getRobotCommander().calibrationScoreSequence());

		joystick.L1.onTrue(robot.getRobotCommander().getIntakeStateHandler().close());
		joystick.getAxisAsButton(Axis.LEFT_TRIGGER).onTrue(robot.getRobotCommander().getIntakeStateHandler().intake());
	}

	private static void applyRobotCommanderCalibrationsBinding(SmartJoystick joystick, Robot robot) {
		joystick.A.onTrue(robot.getRobotCommander().driveWith(RobotState.NEUTRAL));
		joystick.B.onTrue(robot.getRobotCommander().driveWith(RobotState.PRE_SCORE));
		joystick.X.onTrue(robot.getRobotCommander().driveWith(RobotState.SCORE));
		joystick.POV_DOWN.onTrue(robot.getRobotCommander().driveWith(RobotState.STAY_IN_PLACE));
		joystick.POV_LEFT
			.onTrue(robot.getRobotCommander().driveWith(RobotState.CALIBRATION_PRE_SCORE, robot.getRobotCommander().calibrationScoreSequence()));
	}

	private static void applyTurretCalibrationBindings(Arm turret, SmartJoystick joystick, double calibrationMaxPower) {
		// Check limits
		joystick.R1.whileTrue(turret.getCommandsBuilder().setPower(() -> joystick.getAxisValue(Axis.LEFT_Y) * calibrationMaxPower));
		turret.getSysIdCalibrator().setAllButtonsForCalibration(joystick);

		joystick.POV_RIGHT.onTrue(turret.getCommandsBuilder().setTargetPosition(Rotation2d.fromDegrees(20)));
		joystick.POV_LEFT.onTrue(turret.getCommandsBuilder().setTargetPosition(Rotation2d.fromDegrees(50)));
	}

	private static void applyPivotCalibrationBindings(Arm pivot, SmartJoystick joystick, double calibrationMaxPower) {
		// Check limits
		joystick.R1.whileTrue(pivot.getCommandsBuilder().setPower(() -> joystick.getAxisValue(Axis.LEFT_Y) * calibrationMaxPower));

		pivot.getSysIdCalibrator().setAllButtonsForCalibration(joystick);

		joystick.POV_RIGHT.onTrue(pivot.getCommandsBuilder().setTargetPosition(Rotation2d.fromDegrees(20)));
		joystick.POV_LEFT.onTrue(pivot.getCommandsBuilder().setTargetPosition(Rotation2d.fromDegrees(50)));
	}

	private static void applyHoodCalibrationBindings(Arm hood, SmartJoystick joystick, double calibrationMaxPower) {
		// Check limits
		joystick.R1.whileTrue(
			hood.getCommandsBuilder()
				.setPower(
					() -> joystick.getAxisValue(Axis.LEFT_Y) * calibrationMaxPower + (hood.getKgVoltage() / BatteryUtil.getCurrentVoltage())
				)
		);

		hood.getSysIdCalibrator().setAllButtonsForCalibration(joystick);

		joystick.POV_RIGHT.onTrue(hood.getCommandsBuilder().setTargetPosition(Rotation2d.fromDegrees(100)));
		joystick.POV_LEFT.onTrue(hood.getCommandsBuilder().setTargetPosition(Rotation2d.fromDegrees(5)));
	}

	public static void applyIntakeRollerCalibrationsBindings(SmartJoystick joystick, Robot robot) {
		joystick.POV_UP.onTrue(robot.getIntakeRoller().getCommandsBuilder().setVoltage(9));
		joystick.POV_DOWN.onTrue(robot.getIntakeRoller().getCommandsBuilder().setVoltage(-6));
	}

	private static void applyMagazineCalibrationBindings(Roller magazine, SmartJoystick joystick, double maxCalibrationPower) {
		joystick.X.onTrue(magazine.getCommandsBuilder().setPower(0.5 * maxCalibrationPower));
		joystick.Y.onTrue(magazine.getCommandsBuilder().setPower(-0.5 * maxCalibrationPower));
	}

	private static void applyConveyorCalibrationBindings(Roller conveyor, SmartJoystick joystick, double maxCalibrationPower) {
		joystick.X.onTrue(conveyor.getCommandsBuilder().setPower(0.5 * maxCalibrationPower));
		joystick.Y.onTrue(conveyor.getCommandsBuilder().setPower(-0.5 * maxCalibrationPower));
	}

}
