package frc;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.constants.field.enums.Branch;
import frc.constants.field.enums.CoralStation;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.states.DriveRelative;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;

import java.util.Optional;

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
		usedJoystick.L1.whileTrue(robot.getSwerve().getCommandsBuilder().driveByDriversInputs(
			SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH).withDriveRelative(DriveRelative.ROBOT_RELATIVE)));
		usedJoystick.R1.whileTrue(robot.getSwerve().getCommandsBuilder().driveByDriversInputs(
			SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.REEF)));
		usedJoystick.BACK.whileTrue(robot.getSwerve().getCommandsBuilder().driveByDriversInputs(
			SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.CORAL_STATION)));
		usedJoystick.START.whileTrue(robot.getSwerve().getCommandsBuilder().driveByDriversInputs(
			SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.ALGI_REMOVE)));

//		usedJoystick.POV_LEFT
//			.onTrue(new InstantCommand(() -> robot.getSwerve().getStateHandler().setCoralStationSupplier(() -> Optional.of(CoralStation.LEFT))));
//		usedJoystick.POV_RIGHT
//			.onTrue(new InstantCommand(() -> robot.getSwerve().getStateHandler().setCoralStationSupplier(() -> Optional.of(CoralStation.RIGHT))));
		usedJoystick.POV_UP
			.onTrue(new InstantCommand(() -> robot.getSwerve().getStateHandler().setBranchSupplier(() -> Optional.of(Branch.G))));
		usedJoystick.POV_RIGHT
			.onTrue(new InstantCommand(() -> robot.getSwerve().getStateHandler().setBranchSupplier(() -> Optional.of(Branch.F))));
		usedJoystick.POV_LEFT
			.onTrue(new InstantCommand(() -> robot.getSwerve().getStateHandler().setBranchSupplier(() -> Optional.of(Branch.C))));
		usedJoystick.POV_DOWN
			.onTrue(new InstantCommand(() -> robot.getSwerve().getStateHandler().setBranchSupplier(() -> Optional.of(Branch.A))));
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
