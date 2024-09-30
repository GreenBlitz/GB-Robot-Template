package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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


		usedJoystick.START.onTrue(new InstantCommand(() -> robot.getSwerve().setHeading(new Rotation2d())));
		robot.getSwerve()
			.setDefaultCommand(
				robot.getSwerve()
					.getCommandsBuilder()
					.driveBySavedState(
						() -> usedJoystick.getAxisValue(Axis.LEFT_Y),
						() -> usedJoystick.getAxisValue(Axis.LEFT_X),
						() -> usedJoystick.getAxisValue(Axis.RIGHT_X)
					)
			);
	}

	private static void secondJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SECOND_JOYSTICK;
		// bindings...

		usedJoystick.A.onTrue(robot.getSupersturctrue().setState(RobotState.IDLE));
		usedJoystick.B.onTrue(robot.getSupersturctrue().setState(RobotState.PRE_SPEAKER));
		usedJoystick.X.onTrue(robot.getSupersturctrue().setState(RobotState.PRE_AMP));
		usedJoystick.Y.onTrue(robot.getSupersturctrue().setState(RobotState.SPEAKER));
		usedJoystick.POV_UP.onTrue(robot.getSupersturctrue().setState(RobotState.INTAKE));
		usedJoystick.POV_DOWN.onTrue(robot.getSupersturctrue().setState(RobotState.SHOOTER_OUTTAKE));
		usedJoystick.POV_LEFT.onTrue(robot.getSupersturctrue().setState(RobotState.AMP));
		usedJoystick.POV_RIGHT.onTrue(robot.getSupersturctrue().setState(RobotState.TRANSFER_SHOOTER_TO_ARM));

		usedJoystick.L1.whileTrue(robot.getRoller().getCommandsBuilder().moveByPower(() -> usedJoystick.getAxisValue(Axis.LEFT_Y) * 0.7));
		usedJoystick.R1.whileTrue(robot.getFunnel().getCommandsBuilder().setPower(() -> usedJoystick.getAxisValue(Axis.RIGHT_Y) * 0.7));
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
