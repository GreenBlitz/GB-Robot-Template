package frc;

import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.subsystems.endeffector.EndEffectorCommandsBuilder;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.robot.subsystems.swerve.Swerve;

import java.util.function.DoubleSupplier;

public class JoysticksBindings {

	private static final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(JoystickPorts.MAIN);
	private static final SmartJoystick SECOND_JOYSTICK = new SmartJoystick(JoystickPorts.SECOND);
	private static final SmartJoystick THIRD_JOYSTICK = new SmartJoystick(JoystickPorts.THIRD);
	private static final SmartJoystick FOURTH_JOYSTICK = new SmartJoystick(JoystickPorts.FOURTH);
	private static final SmartJoystick FIFTH_JOYSTICK = new SmartJoystick(JoystickPorts.FIFTH);
	private static final SmartJoystick SIXTH_JOYSTICK = new SmartJoystick(JoystickPorts.SIXTH);

	public static void configureBindings(Robot robot) {
		mainJoystickButtons(robot);
		secondJoystickButtons(robot);
		thirdJoystickButtons(robot);
		fourthJoystickButtons(robot);
		fifthJoystickButtons(robot);
		sixthJoystickButtons(robot);
	}

	public static void setDriversInputsToSwerve(Swerve swerve) {
		if (MAIN_JOYSTICK.isConnected()) {
			swerve.setDriversPowerInputs(
				new ChassisPowers(
					MAIN_JOYSTICK.getAxisValue(Axis.LEFT_Y),
					MAIN_JOYSTICK.getAxisValue(Axis.LEFT_X),
					MAIN_JOYSTICK.getAxisValue(Axis.RIGHT_X)
				)
			);
		} else if (THIRD_JOYSTICK.isConnected()) {
			swerve.setDriversPowerInputs(
				new ChassisPowers(
					THIRD_JOYSTICK.getAxisValue(Axis.LEFT_Y),
					THIRD_JOYSTICK.getAxisValue(Axis.LEFT_X),
					THIRD_JOYSTICK.getAxisValue(Axis.RIGHT_X)
				)
			);
		} else {
			swerve.setDriversPowerInputs(new ChassisPowers(0, 0, 0));
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
		EndEffectorCommandsBuilder endEffectorCommandsBuilder = robot.getEndEffector().getCommandsBuilder();

		usedJoystick.B.whileTrue(endEffectorCommandsBuilder.setPower(() -> Math.max(robot.getEndEffector().getPower() - 0.01, -1)));
		usedJoystick.X.whileTrue(endEffectorCommandsBuilder.setPower(() -> Math.min(robot.getEndEffector().getPower() + 0.01, 1)));
		usedJoystick.A.whileTrue(endEffectorCommandsBuilder.setPower(() -> Math.max(robot.getEndEffector().getPower() - 0.05, -1)));
		usedJoystick.Y.whileTrue(endEffectorCommandsBuilder.setPower(() -> Math.min(robot.getEndEffector().getPower() + 0.05, 1)));

		usedJoystick.POV_RIGHT.whileTrue(endEffectorCommandsBuilder.setPower(() -> Math.max(robot.getEndEffector().getPower() - 0.1, -1)));
		usedJoystick.POV_LEFT.whileTrue(endEffectorCommandsBuilder.setPower(() -> Math.min(robot.getEndEffector().getPower() + 0.1, 1)));
		usedJoystick.POV_DOWN.whileTrue(endEffectorCommandsBuilder.setPower(() -> Math.max(robot.getEndEffector().getPower() - 0.5, -1)));
		usedJoystick.POV_UP.whileTrue(endEffectorCommandsBuilder.setPower(() -> Math.min(robot.getEndEffector().getPower() + 0.5, 1)));



	}

}
