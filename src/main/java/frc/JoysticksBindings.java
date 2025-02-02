package frc;

import frc.constants.field.enums.CoralStation;
import frc.constants.field.enums.ReefSide;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.RobotState;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.robot.subsystems.swerve.Swerve;

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
		usedJoystick.A.onTrue(robot.getRobotCommander().setState(RobotState.DRIVE));
		usedJoystick.B.onTrue(robot.getRobotCommander().setState(RobotState.INTAKE));
		usedJoystick.X.onTrue(robot.getRobotCommander().setState(RobotState.OUTTAKE));
		usedJoystick.Y.onTrue(robot.getRobotCommander().setState(RobotState.ALIGN_REEF));
		usedJoystick.POV_DOWN.onTrue(robot.getRobotCommander().setState(RobotState.PRE_L1));
		usedJoystick.POV_LEFT.onTrue(robot.getRobotCommander().setState(RobotState.PRE_L2));
		usedJoystick.POV_UP.onTrue(robot.getRobotCommander().setState(RobotState.PRE_L3));
		usedJoystick.POV_RIGHT.onTrue(robot.getRobotCommander().setState(RobotState.PRE_L4));
		usedJoystick.L1.onTrue(robot.getRobotCommander().setState(RobotState.L1));
		usedJoystick.R1.onTrue(robot.getRobotCommander().setState(RobotState.L2));
		usedJoystick.START.onTrue(robot.getRobotCommander().setState(RobotState.L3));
		usedJoystick.L3.onTrue(robot.getRobotCommander().setState(RobotState.L4));

		robot.getSwerve().getStateHandler().setBranchSupplier(() -> Optional.of(ScoringHelpers.targetBranch));
		robot.getSwerve().getStateHandler().setCoralStationSupplier(() -> Optional.of(CoralStation.RIGHT));
		robot.getSwerve().getStateHandler().setRobotPoseSupplier(() -> robot.getPoseEstimator().getEstimatedPose());
		robot.getSwerve().getStateHandler().setReefSideSupplier(() -> Optional.of(ReefSide.F));
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
