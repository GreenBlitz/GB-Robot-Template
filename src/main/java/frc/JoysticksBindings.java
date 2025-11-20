package frc;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
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
		applyHoodCalibrationBindings(robot.getHood(), usedJoystick, 1);
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

}
