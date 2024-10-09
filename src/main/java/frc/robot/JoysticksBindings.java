package frc.robot;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.elbow.ElbowState;
import frc.robot.subsystems.flywheel.FlywheelState;
import frc.robot.subsystems.pivot.PivotState;
import frc.robot.subsystems.wrist.WristState;
import frc.robot.superstructure.Tolerances;
import frc.utils.joysticks.JoystickPorts;
import frc.utils.joysticks.SmartJoystick;
import org.littletonrobotics.junction.Logger;

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
		
		usedJoystick.Y.onTrue(robot.getSuperstructure().flywheelStateHandler.setState(FlywheelState.PRE_SPEAKER)
				.alongWith(new RunCommand(() -> Logger.recordOutput(
						"is FLYWHEEL in velcity",
						robot.getFlywheel().isAtVelocities(
								FlywheelState.PRE_SPEAKER.getRightVelocity(), FlywheelState.PRE_SPEAKER.getLeftVelocity(),
								Tolerances.FLYWHEEL_VELOCITY_PER_SECOND)))));
		usedJoystick.X.onTrue(robot.getSuperstructure().flywheelStateHandler.setState(FlywheelState.DEFAULT)
				.alongWith(new RunCommand(() -> Logger.recordOutput(
						"is FLYWHEEL in velcity",
						robot.getFlywheel().isAtVelocities(
								FlywheelState.DEFAULT.getRightVelocity(), FlywheelState.DEFAULT.getLeftVelocity(),
								Tolerances.FLYWHEEL_VELOCITY_PER_SECOND)))));
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
