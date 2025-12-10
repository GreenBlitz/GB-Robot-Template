package frc;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.ChassisPowers;

public class JoysticksBindings {

	private static final double POWER_SCALING = 0.7;

	private static final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(JoystickPorts.MAIN);
	private static final SmartJoystick SECOND_JOYSTICK = new SmartJoystick(JoystickPorts.SECOND);
	private static final SmartJoystick THIRD_JOYSTICK = new SmartJoystick(JoystickPorts.THIRD);
	private static final SmartJoystick FOURTH_JOYSTICK = new SmartJoystick(JoystickPorts.FOURTH);
	private static final SmartJoystick FIFTH_JOYSTICK = new SmartJoystick(JoystickPorts.FIFTH);
	private static final SmartJoystick SIXTH_JOYSTICK = new SmartJoystick(JoystickPorts.SIXTH);

	public static final ChassisPowers chassisDriverInputs = new ChassisPowers();

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
			chassisDriverInputs.xPower = MAIN_JOYSTICK.getAxisValue(Axis.LEFT_Y) * POWER_SCALING;
			chassisDriverInputs.yPower = MAIN_JOYSTICK.getAxisValue(Axis.LEFT_X) * POWER_SCALING;
			chassisDriverInputs.rotationalPower = MAIN_JOYSTICK.getAxisValue(Axis.RIGHT_X) * POWER_SCALING;
		} else if (THIRD_JOYSTICK.isConnected()) {
			chassisDriverInputs.xPower = THIRD_JOYSTICK.getAxisValue(Axis.LEFT_Y) * POWER_SCALING;
			chassisDriverInputs.yPower = THIRD_JOYSTICK.getAxisValue(Axis.LEFT_X) * POWER_SCALING;
			chassisDriverInputs.rotationalPower = THIRD_JOYSTICK.getAxisValue(Axis.RIGHT_X) * POWER_SCALING;
		} else {
			chassisDriverInputs.xPower = 0;
			chassisDriverInputs.yPower = 0;
			chassisDriverInputs.rotationalPower = 0;
		}
	}

	private static void mainJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = MAIN_JOYSTICK;
		// bindings...
		switch (Robot.TEAM_NUMBER) {
			case 0 -> {
				usedJoystick.R1.whileTrue(
					new RunCommand(
						() -> robot.getEndEffector().getClosedLoopController().setReference(10, SparkBase.ControlType.kPosition),
						robot.getSubsystem()
					)
				);
				usedJoystick.L1.whileTrue(
					new RunCommand(
						() -> robot.getEndEffector().getClosedLoopController().setReference(0, SparkBase.ControlType.kPosition),
						robot.getSubsystem()
					)
				);
			}
			case 1 -> {
				usedJoystick.R1.whileTrue(
					new RunCommand(
						() -> robot.getEndEffector().getClosedLoopController().setReference(0.1, SparkBase.ControlType.kPosition),
						robot.getSubsystem()
					)
				);
				usedJoystick.L1.whileTrue(
					new RunCommand(
						() -> robot.getEndEffector().getClosedLoopController().setReference(2.5, SparkBase.ControlType.kPosition),
						robot.getSubsystem()
					)
				);
			}
			case 2 -> {
				usedJoystick.getAxisAsButton(Axis.LEFT_TRIGGER)
					.onTrue(
						new RunCommand(() -> robot.getEndEffector().set(usedJoystick.getAxisValue(Axis.LEFT_TRIGGER)), robot.getSubsystem())
					);
				usedJoystick.getAxisAsButton(Axis.RIGHT_TRIGGER)
					.onTrue(
						new RunCommand(() -> robot.getEndEffector().set(-usedJoystick.getAxisValue(Axis.RIGHT_TRIGGER)), robot.getSubsystem())
					);
			}
			case 3 -> {
				usedJoystick.R1.whileTrue(
					new RunCommand(
						() -> robot.getEndEffector().getClosedLoopController().setReference(0, SparkBase.ControlType.kPosition),
						robot.getSubsystem()
					)
				);
				usedJoystick.L1.whileTrue(
					new RunCommand(
						() -> robot.getEndEffector().getClosedLoopController().setReference(5.1, SparkBase.ControlType.kPosition),
						robot.getSubsystem()
					)
				);
			}
		}
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
