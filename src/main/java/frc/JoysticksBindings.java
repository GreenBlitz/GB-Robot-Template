package frc;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.field.Field;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.RobotCommander;
import frc.robot.statemachine.RobotState;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.utils.utilcommands.ExecuteEndCommand;

import java.util.Set;

public class JoysticksBindings {

	private static final double NOTE_IN_RUMBLE_TIME_SECONDS = 0.5;
	private static final double NOTE_IN_RUMBLE_POWER = 0.4;

	private static final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(JoystickPorts.MAIN);
	private static final SmartJoystick SECOND_JOYSTICK = new SmartJoystick(JoystickPorts.SECOND);
	private static final SmartJoystick THIRD_JOYSTICK = new SmartJoystick(JoystickPorts.THIRD);
	private static final SmartJoystick FOURTH_JOYSTICK = new SmartJoystick(JoystickPorts.FOURTH);
//	private static final SmartJoystick FIFTH_JOYSTICK = new SmartJoystick(JoystickPorts.FIFTH);
//	private static final SmartJoystick SIXTH_JOYSTICK = new SmartJoystick(JoystickPorts.SIXTH);


	private static final ChassisPowers chassisDriverInputs = new ChassisPowers();

	public static void configureBindings(Robot robot) {
		robot.getSwerve().setDriversPowerInputs(chassisDriverInputs);

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

	private static Command noteInRumble(SmartJoystick joystick) {
		return new ExecuteEndCommand(
			() -> joystick.setRumble(GenericHID.RumbleType.kBothRumble, NOTE_IN_RUMBLE_POWER),
			() -> joystick.stopRumble(GenericHID.RumbleType.kBothRumble)
		).withTimeout(NOTE_IN_RUMBLE_TIME_SECONDS);
	}

	private static Command reefActionChooser(Robot robot) {
		return new DeferredCommand(
			() -> robot.getRobotCommander().getSuperstructure().isCoralIn()
				? Field.isOnBlueSide(robot.getPoseEstimator().getEstimatedPose().getTranslation()) == Field.isFieldConventionAlliance()
					? (ScoringHelpers.isAutoAlgaeRemoveActivated
						? robot.getRobotCommander().reefAutomationThenAlgaeRemove()
						: robot.getRobotCommander().reefAutomation())
					: new InstantCommand()
				: new InstantCommand(() -> ScoringHelpers.setClosetReefSideTarget(robot))
					.andThen(robot.getRobotCommander().driveWith(RobotState.ALGAE_REMOVE)),
			Set.of(
				robot.getRobotCommander(),
				robot.getRobotCommander().getSuperstructure(),
				robot.getSwerve(),
				robot.getElevator(),
				robot.getArm(),
				robot.getEndEffector(),
				robot.getLifter(),
				robot.getSolenoid(),
				robot.getPivot(),
				robot.getRollers(),
				robot.getRobotCommander().getLedStateHandler()
			)
		);
	}

	private static Command closeReefActionChooser(Robot robot) {
		return new SequentialCommandGroup(
			new InstantCommand(() -> ScoringHelpers.setClosetReefSideTarget(robot)),
			new DeferredCommand(
				() -> reefActionChooser(robot),
				Set.of(
					robot.getRobotCommander(),
					robot.getRobotCommander().getSuperstructure(),
					robot.getSwerve(),
					robot.getElevator(),
					robot.getArm(),
					robot.getEndEffector(),
					robot.getLifter(),
					robot.getPivot(),
					robot.getRollers(),
					robot.getSolenoid(),
					robot.getRobotCommander().getLedStateHandler()
				)
			)
		);
	}

	private static Command driveActionChooser(Robot robot) {
		return new InstantCommand(() -> {
			RobotCommander robotCommander = robot.getRobotCommander();
			RobotState state = robotCommander.getCurrentState();
			Command command;
			if (state == RobotState.ALGAE_REMOVE || state == RobotState.PRE_NET) {
				robotCommander.driveWith(RobotState.HOLD_ALGAE).schedule();
				return;
			} else {
				command = Commands.none();
			}
			command.andThen(robotCommander.driveWith(RobotState.DRIVE)).schedule();
		});
	}

	private static Command netActionChooser(Robot robot) {
		return new InstantCommand(() -> {
			RobotCommander robotCommander = robot.getRobotCommander();
			RobotState state = robotCommander.getCurrentState();
			Command command;

			if (state == RobotState.PRE_NET && robotCommander.netAssist) {
				robotCommander.netAssist = false;
				command = robotCommander.driveWith(RobotState.PRE_NET);
			} else if (state == RobotState.NET || state == RobotState.PRE_NET) {
				command = robotCommander.driveWith(RobotState.NET);
			} else {
				robotCommander.netAssist = true;
				command = robotCommander.netAutomation();
			}
			command.schedule();
		});
	}

	private static Command processorActionChooser(Robot robot) {
		RobotCommander robotCommander = robot.getRobotCommander();

		return new ConditionalCommand(
			robotCommander.driveWith(RobotState.ALGAE_OUTTAKE_FROM_INTAKE),
			robotCommander.processorAutomation(),
			() -> robotCommander.getSuperstructure().isAlgaeInAlgaeIntake()
		);
	}

	private static Command algaeOuttakeActionChooser(Robot robot) {
		RobotCommander robotCommander = robot.getRobotCommander();

		return new DeferredCommand(
			() -> robotCommander.driveWith(
				robotCommander.getSuperstructure().isAlgaeInAlgaeIntake()
					? RobotState.ALGAE_OUTTAKE_FROM_INTAKE
					: RobotState.ALGAE_OUTTAKE_FROM_END_EFFECTOR
			),
			Set.of(
				robotCommander,
				robotCommander.getSuperstructure(),
				robot.getSwerve(),
				robot.getElevator(),
				robot.getArm(),
				robot.getEndEffector(),
				robot.getLifter(),
				robot.getSolenoid(),
				robot.getPivot(),
				robot.getRollers()
			)
		);
	}

	public static void updateChassisDriverInputs() {
		if (MAIN_JOYSTICK.isConnected()) {
			chassisDriverInputs.xPower = MAIN_JOYSTICK.getAxisValue(Axis.LEFT_Y);
			chassisDriverInputs.yPower = MAIN_JOYSTICK.getAxisValue(Axis.LEFT_X);
			chassisDriverInputs.rotationalPower = MAIN_JOYSTICK.getAxisValue(Axis.RIGHT_X);
		} else if (THIRD_JOYSTICK.isConnected()) {
			chassisDriverInputs.xPower = THIRD_JOYSTICK.getAxisValue(Axis.LEFT_Y);
			chassisDriverInputs.yPower = THIRD_JOYSTICK.getAxisValue(Axis.LEFT_X);
			chassisDriverInputs.rotationalPower = THIRD_JOYSTICK.getAxisValue(Axis.RIGHT_X);
		} else {
			chassisDriverInputs.xPower = 0;
			chassisDriverInputs.yPower = 0;
			chassisDriverInputs.rotationalPower = 0;
		}
	}

	private static void mainJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = MAIN_JOYSTICK;
		// bindings...
		usedJoystick.getAxisAsButton(Axis.RIGHT_TRIGGER).onTrue(closeReefActionChooser(robot));

		usedJoystick.X.onTrue(robot.getRobotCommander().intakeAutomation());
		usedJoystick.L1.onTrue(robot.getRobotCommander().driveWith(RobotState.ALGAE_INTAKE));

		usedJoystick.getAxisAsButton(Axis.LEFT_TRIGGER).onTrue(robot.getRobotCommander().driveWith(RobotState.INTAKE));

		usedJoystick.R1.onTrue(netActionChooser(robot));

		usedJoystick.Y.onTrue(robot.getRobotCommander().driveWith(RobotState.CORAL_OUTTAKE));
		usedJoystick.X.onTrue(algaeOuttakeActionChooser(robot));
		usedJoystick.B.onTrue(processorActionChooser(robot));


		usedJoystick.POV_LEFT.onTrue(robot.getRobotCommander().driveWith(RobotState.PRE_CLIMB.activateSwerve(true)));
		usedJoystick.POV_UP.onTrue(robot.getRobotCommander().driveWith(RobotState.PRE_CLIMB.activateSwerve(false)));
		usedJoystick.POV_DOWN.onTrue(robot.getRobotCommander().driveWith(RobotState.CLIMB_WITH_LIMIT_SWITCH));
		usedJoystick.A.onTrue(driveActionChooser(robot));

		usedJoystick.START.whileTrue(robot.getRobotCommander().driveWith(RobotState.MANUAL_CLIMB));
		usedJoystick.BACK.whileTrue(robot.getRobotCommander().driveWith(RobotState.EXIT_CLIMB));
	}

	private static void secondJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SECOND_JOYSTICK;
		// bindings...

		usedJoystick.POV_LEFT.onTrue(new InstantCommand(() -> robot.getRobotCommander().keepAlgaeInIntake = false));
		usedJoystick.POV_RIGHT.onTrue(new InstantCommand(() -> robot.getRobotCommander().keepAlgaeInIntake = true));

		usedJoystick.A.onTrue(new InstantCommand(() -> ScoringHelpers.targetScoreLevel = ScoreLevel.L1));
		usedJoystick.B.onTrue(new InstantCommand(() -> ScoringHelpers.targetScoreLevel = ScoreLevel.L2));
		usedJoystick.X.onTrue(new InstantCommand(() -> ScoringHelpers.targetScoreLevel = ScoreLevel.L3));
		usedJoystick.Y.onTrue(new InstantCommand(() -> ScoringHelpers.targetScoreLevel = ScoreLevel.L4));

		usedJoystick.R1.onTrue(new InstantCommand(() -> ScoringHelpers.isLeftBranch = false));
		usedJoystick.L1.onTrue(new InstantCommand(() -> ScoringHelpers.isLeftBranch = true));

		usedJoystick.getAxisAsButton(Axis.RIGHT_TRIGGER).onTrue(robot.getRobotCommander().driveWith(RobotState.INTAKE.activateSwerve(false)));
		usedJoystick.POV_UP.onTrue(new InstantCommand(() -> ScoringHelpers.isAutoAlgaeRemoveActivated = true));

		usedJoystick.L3.onTrue(robot.getRobotCommander().driveWith(RobotState.PRE_CLIMB));
	}

	private static void thirdJoystickButtons(Robot robot) {
//		SmartJoystick usedJoystick = THIRD_JOYSTICK;
		// bindings...

		// robot.getSwerve().applyCalibrationBindings(usedJoystick, () -> robot.getPoseEstimator().getEstimatedPose());
	}

	private static void fourthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = FOURTH_JOYSTICK;
		// bindings...

		robot.getElevator().applyCalibrationBindings(usedJoystick);
	}

	private static void fifthJoystickButtons(Robot robot) {
//		SmartJoystick usedJoystick = FIFTH_JOYSTICK;
		// bindings...

//		robot.getRobotCommander().getSuperstructure().getAlgaeIntakeStateHandler().applyCalibrationBindings(usedJoystick);
//		robot.getRobotCommander().getSuperstructure().getClimbStateHandler().applyCalibrationBindings(usedJoystick);
	}

	private static void sixthJoystickButtons(Robot robot) {
//		SmartJoystick usedJoystick = SIXTH_JOYSTICK;

//		robot.getArm().applyCalibrationBindings(usedJoystick);
//		robot.getEndEffector().applyCalibrationsBindings(usedJoystick);
	}

}
