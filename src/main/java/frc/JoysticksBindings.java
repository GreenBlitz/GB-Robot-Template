package frc;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.constants.field.enums.Branch;
import frc.constants.field.enums.ReefSide;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;

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
		usedJoystick.L1.whileTrue(
			robot.getSwerve()
				.getCommandsBuilder()
				.driveByState(
					() -> new ChassisPowers(
						usedJoystick.getAxisValue(Axis.LEFT_Y),
						usedJoystick.getAxisValue(Axis.LEFT_X),
						usedJoystick.getSensitiveAxisValue(Axis.RIGHT_X)
					),
					SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH)
				)
		);
		usedJoystick.POV_LEFT
			.onTrue(new InstantCommand(() -> robot.getSwerve().getStateHandler().setBranchSupplier(() -> java.util.Optional.of(Branch.A))));
		usedJoystick.POV_RIGHT
			.onTrue(new InstantCommand(() -> robot.getSwerve().getStateHandler().setBranchSupplier(() -> java.util.Optional.of(Branch.B))));
		usedJoystick.POV_UP
			.onTrue(new InstantCommand(() -> robot.getSwerve().getStateHandler().setBranchSupplier(() -> java.util.Optional.of(Branch.C))));
		usedJoystick.A
			.onTrue(new InstantCommand(() -> robot.getSwerve().getStateHandler().setReefSideSupplier(() -> java.util.Optional.of(ReefSide.A))));
		usedJoystick.B
			.onTrue(new InstantCommand(() -> robot.getSwerve().getStateHandler().setReefSideSupplier(() -> java.util.Optional.of(ReefSide.B))));
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
