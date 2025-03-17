package frc;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.RobotState;
import frc.robot.statemachine.StateMachineConstants;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.robot.subsystems.swerve.Swerve;
import frc.utils.utilcommands.ExecuteEndCommand;

import java.util.Set;

public class JoysticksBindings {

	private static final double NOTE_IN_RUMBLE_TIME_SECONDS = 0.5;
	private static final double NOTE_IN_RUMBLE_POWER = 0.4;

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

		Trigger noteIn = new Trigger(robot.getRobotCommander().getSuperstructure()::isCoralIn);
		noteIn.onTrue(noteInRumble(MAIN_JOYSTICK).alongWith(noteInRumble(SECOND_JOYSTICK)));

		Trigger noteOut = new Trigger(() -> !robot.getRobotCommander().getSuperstructure().isCoralIn());
		noteOut.onTrue(noteInRumble(MAIN_JOYSTICK).alongWith(noteInRumble(SECOND_JOYSTICK)));
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

	private static Command noteInRumble(SmartJoystick joystick) {
		return new ExecuteEndCommand(
			() -> joystick.setRumble(GenericHID.RumbleType.kBothRumble, NOTE_IN_RUMBLE_POWER),
			() -> joystick.stopRumble(GenericHID.RumbleType.kBothRumble)
		).withTimeout(NOTE_IN_RUMBLE_TIME_SECONDS);
	}

	private static Command reefActionChooser(Robot robot) {
		return new DeferredCommand(
			() -> robot.getRobotCommander().getSuperstructure().isCoralIn()
				? robot.getRobotCommander().autoScore()
				: new InstantCommand(() -> ScoringHelpers.setClosetReefSideTarget(robot))
					.andThen(robot.getRobotCommander().setState(RobotState.ALGAE_REMOVE)),
			Set.of(
				robot.getRobotCommander(),
				robot.getRobotCommander().getSuperstructure(),
				robot.getSwerve(),
				robot.getElevator(),
				robot.getArm(),
				robot.getEndEffector(),
				robot.getLifter(),
				robot.getSolenoid()
			)
		);
	}

	private static Command closeReefActionChooser(Robot robot) {
		return new DeferredCommand(
			() -> new SequentialCommandGroup(new InstantCommand(() -> ScoringHelpers.setClosetReefSideTarget(robot)), reefActionChooser(robot)),
			Set.of(
				robot.getRobotCommander(),
				robot.getRobotCommander().getSuperstructure(),
				robot.getSwerve(),
				robot.getElevator(),
				robot.getArm(),
				robot.getEndEffector(),
				robot.getLifter(),
				robot.getSolenoid()
			)
		);
	}

	private static void mainJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = MAIN_JOYSTICK;
		// bindings...
		usedJoystick.getAxisAsButton(Axis.RIGHT_TRIGGER).onTrue(closeReefActionChooser(robot));

		usedJoystick.L1.onTrue(robot.getRobotCommander().setState(RobotState.INTAKE_WITH_AIM_ASSIST));
		usedJoystick.getAxisAsButton(Axis.LEFT_TRIGGER).onTrue(robot.getRobotCommander().setState(RobotState.INTAKE_WITHOUT_AIM_ASSIST));

		usedJoystick.R1.onTrue(robot.getRobotCommander().fullyNet());

		usedJoystick.Y.onTrue(robot.getRobotCommander().setState(RobotState.CORAL_OUTTAKE));
		usedJoystick.X.onTrue(robot.getRobotCommander().setState(RobotState.ALGAE_OUTTAKE));
		usedJoystick.B.onTrue(robot.getRobotCommander().setState(RobotState.PROCESSOR_SCORE));

		usedJoystick.POV_LEFT.onTrue(robot.getRobotCommander().setState(RobotState.PRE_CLIMB_WITH_AIM_ASSIST));
		usedJoystick.POV_UP.onTrue(robot.getRobotCommander().setState(RobotState.PRE_CLIMB_WITHOUT_AIM_ASSIST));
		usedJoystick.POV_DOWN.onTrue(robot.getRobotCommander().setState(RobotState.CLIMB));
		usedJoystick.A.onTrue(robot.getRobotCommander().setState(RobotState.DRIVE));

		Command climbUp = robot.getLifter().getCommandsBuilder().setPower(StateMachineConstants.POWER_FOR_MANUAL_CLIMB);
		climbUp.addRequirements(robot.getRobotCommander(), robot.getRobotCommander().getSuperstructure());

		usedJoystick.START.whileTrue(climbUp);
	}

	private static void secondJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SECOND_JOYSTICK;
		// bindings...

		usedJoystick.A.onTrue(new InstantCommand(() -> ScoringHelpers.targetScoreLevel = ScoreLevel.L1));
		usedJoystick.B.onTrue(new InstantCommand(() -> ScoringHelpers.targetScoreLevel = ScoreLevel.L2));
		usedJoystick.X.onTrue(new InstantCommand(() -> ScoringHelpers.targetScoreLevel = ScoreLevel.L3));
		usedJoystick.Y.onTrue(new InstantCommand(() -> ScoringHelpers.targetScoreLevel = ScoreLevel.L4));

		usedJoystick.R1.onTrue(new InstantCommand(() -> ScoringHelpers.isLeftBranch = false));
		usedJoystick.L1.onTrue(new InstantCommand(() -> ScoringHelpers.isLeftBranch = true));

		usedJoystick.getAxisAsButton(Axis.RIGHT_TRIGGER).onTrue(robot.getRobotCommander().setState(RobotState.ALGAE_OUTTAKE));
		usedJoystick.L3.onTrue(robot.getRobotCommander().setState(RobotState.PRE_CLIMB_WITHOUT_AIM_ASSIST));
	}

	private static void thirdJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = THIRD_JOYSTICK;
		// bindings...

		robot.getSwerve().applyCalibrationBindings(usedJoystick, () -> robot.getPoseEstimator().getEstimatedPose());
	}

	private static void fourthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = FOURTH_JOYSTICK;
		// bindings...

		robot.getElevator().applyCalibrationBindings(usedJoystick);
	}

	private static void fifthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = FIFTH_JOYSTICK;
		// bindings...

		robot.getRobotCommander().getSuperstructure().getClimbStateHandler().applyCalibrationBindings(usedJoystick);
	}

	private static void sixthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SIXTH_JOYSTICK;

		robot.getArm().applyCalibrationBindings(usedJoystick);
//		robot.getEndEffector().applyCalibrationsBindings(usedJoystick);
	}

}
