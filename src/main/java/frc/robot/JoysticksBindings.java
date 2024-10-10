package frc.robot;

import frc.robot.constants.IDs;
import frc.robot.superstructure.RobotState;
import frc.utils.joysticks.Axis;
import frc.utils.joysticks.JoystickPorts;
import frc.utils.joysticks.SmartJoystick;

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

	private static void mainJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = MAIN_JOYSTICK;
		// bindings...

		robot.getSwerve().setDefaultCommand(robot.getSwerve().getCommandsBuilder().driveBySavedState(
				() -> usedJoystick.getAxisValue(Axis.LEFT_Y),
				() -> usedJoystick.getAxisValue(Axis.LEFT_X),
				() -> usedJoystick.getAxisValue(Axis.RIGHT_X)
		));

		usedJoystick.A.onTrue(robot.getSuperstructure().setState(RobotState.IDLE));
		usedJoystick.X.onTrue(robot.getSuperstructure().setState(RobotState.AMP));
		usedJoystick.Y.onTrue(robot.getSuperstructure().setState(RobotState.SPEAKER));
		usedJoystick.POV_UP.onTrue(robot.getSuperstructure().setState(RobotState.SHOOTER_INTAKE));
		usedJoystick.POV_DOWN.onTrue(robot.getSuperstructure().setState(RobotState.INTAKE_OUTTAKE));
		usedJoystick.B.onTrue(robot.getSuperstructure().setState(RobotState.ARM_OUTTAKE));
		usedJoystick.R1.onTrue(robot.getSuperstructure().setState(RobotState.TRANSFER_SHOOTER_TO_ARM));
		usedJoystick.L1.onTrue(robot.getSuperstructure().setState(RobotState.TRANSFER_ARM_TO_SHOOTER));

		usedJoystick.getAxisAsButton(Axis.LEFT_TRIGGER, 0.1).whileTrue(
				robot.getFunnel().getCommandsBuilder().setPower(() -> -usedJoystick.getAxisValue(Axis.LEFT_TRIGGER))
		);
		usedJoystick.getAxisAsButton(Axis.RIGHT_TRIGGER, 0.1).whileTrue(
				robot.getFunnel().getCommandsBuilder().setPower(() -> usedJoystick.getAxisValue(Axis.RIGHT_TRIGGER))
		);
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

}
