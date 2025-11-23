package frc;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.utils.battery.BatteryUtil;

public class JoysticksBindings {

	private static final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(JoystickPorts.MAIN);
	private static final SmartJoystick SECOND_JOYSTICK = new SmartJoystick(JoystickPorts.SECOND);
	private static final SmartJoystick THIRD_JOYSTICK = new SmartJoystick(JoystickPorts.THIRD);
	private static final SmartJoystick FOURTH_JOYSTICK = new SmartJoystick(JoystickPorts.FOURTH);
	private static final SmartJoystick FIFTH_JOYSTICK = new SmartJoystick(JoystickPorts.FIFTH);
	private static final SmartJoystick SIXTH_JOYSTICK = new SmartJoystick(JoystickPorts.SIXTH);

	private static final ChassisPowers chassisDriverInputs = new ChassisPowers();

	public static void configureBindings(Robot robot) {
		// Set 'chassisDriverInputs' to swerve...

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

	private static void mainJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = MAIN_JOYSTICK;
		// bindings...
	}

	private static void secondJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SECOND_JOYSTICK;
		// bindings...
	}

	private static void thirdJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = THIRD_JOYSTICK;
		// bindings...
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

	private static void applyTurretCalibrationBindings(Arm turret, SmartJoystick joystick, double calibrationMaxPower) {
		joystick.POV_DOWN.onTrue(new InstantCommand(() -> turret.getCommandsBuilder().setIsSubsystemRunningIndependently(true)));
		joystick.POV_UP.onTrue(new InstantCommand(() -> turret.getCommandsBuilder().setIsSubsystemRunningIndependently(false)));

		// Check limits
		joystick.R1.whileTrue(turret.getCommandsBuilder().setPower(() -> joystick.getAxisValue(Axis.LEFT_Y) * calibrationMaxPower));
		turret.getSysIdCalibrator().setAllButtonsForCalibration(joystick);

		joystick.POV_RIGHT.onTrue(turret.getCommandsBuilder().setTargetPosition(Rotation2d.fromDegrees(20)));
		joystick.POV_LEFT.onTrue(turret.getCommandsBuilder().setTargetPosition(Rotation2d.fromDegrees(50)));
	}

	private static void applyHoodCalibrationBindings(Arm hood, SmartJoystick joystick, double calibrationMaxPower) {
		joystick.POV_DOWN.onTrue(new InstantCommand(() -> hood.getCommandsBuilder().setIsSubsystemRunningIndependently(true)));
		joystick.POV_UP.onTrue(new InstantCommand(() -> hood.getCommandsBuilder().setIsSubsystemRunningIndependently(false)));

		// Check limits
		joystick.R1.whileTrue(
			hood.getCommandsBuilder()
				.setPower(
					() -> joystick.getAxisValue(Axis.LEFT_Y) * calibrationMaxPower + (hood.getKgVoltage() / BatteryUtil.getCurrentVoltage())
				)
		);

		hood.getSysIdCalibrator().setAllButtonsForCalibration(joystick);

		joystick.POV_RIGHT.onTrue(hood.getCommandsBuilder().setTargetPosition(Rotation2d.fromDegrees(20)));
		joystick.POV_LEFT.onTrue(hood.getCommandsBuilder().setTargetPosition(Rotation2d.fromDegrees(50)));
	}

	public static void applyIntakeRollerCalibrationsBindings(SmartJoystick joystick, Robot robot) {
		joystick.L1.onTrue(new InstantCommand(() -> robot.getIntakeRoller().getCommandsBuilder().setIsSubsystemRunningIndependently(true)));
		joystick.L3.onTrue(new InstantCommand(() -> robot.getIntakeRoller().getCommandsBuilder().setIsSubsystemRunningIndependently(false)));

		joystick.POV_UP.onTrue(robot.getIntakeRoller().getCommandsBuilder().setVoltage(6));
		joystick.POV_DOWN.onTrue(robot.getIntakeRoller().getCommandsBuilder().setVoltage(-6));
	}

	private static void applyBellyCalibrationBindings(Roller belly, SmartJoystick joystick, double calibrationMaxPower) {
		joystick.POV_DOWN.onTrue(new InstantCommand(() -> belly.getCommandsBuilder().setIsSubsystemRunningIndependently(true)));
		joystick.POV_UP.onTrue(new InstantCommand(() -> belly.getCommandsBuilder().setIsSubsystemRunningIndependently(false)));

		joystick.POV_RIGHT.onTrue(belly.getCommandsBuilder().rollRotationsAtVoltageForwards(1, 1));
		joystick.POV_LEFT.onTrue(belly.getCommandsBuilder().setVoltage(2));
	}

	private static void applyOmniCalibrationBindings(Roller omni, SmartJoystick joystick, double maxCalibrationPower) {
		joystick.POV_DOWN.onTrue(new InstantCommand(() -> omni.getCommandsBuilder().setIsSubsystemRunningIndependently(true)));
		joystick.POV_UP.onTrue(new InstantCommand(() -> omni.getCommandsBuilder().setIsSubsystemRunningIndependently(false)));

		joystick.X.onTrue(omni.getCommandsBuilder().setPower(0.5));
		joystick.Y.onTrue(omni.getCommandsBuilder().setPower(-0.5));
	}

}
