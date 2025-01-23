package frc;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmStateHandler;

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
//		usedJoystick.B.onTrue(robot.getArm().getCommandsBuilder().moveToTargetPosition(Rotation2d.fromDegrees(35)));
		ArmStateHandler stateHandler = new ArmStateHandler(robot.getArm());
		usedJoystick.A.onTrue(stateHandler.setState(ArmState.CLOSED));
		usedJoystick.B.onTrue(stateHandler.setState(ArmState.INTAKE));
		usedJoystick.X.onTrue(stateHandler.setState(ArmState.L1));
		usedJoystick.Y.onTrue(stateHandler.setState(ArmState.L2));
		usedJoystick.POV_DOWN.onTrue(stateHandler.setState(ArmState.L3));
		usedJoystick.POV_LEFT.onTrue(stateHandler.setState(ArmState.L4));
		usedJoystick.POV_RIGHT.onTrue(stateHandler.setState(ArmState.OUTTAKE));

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
