package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.constants.Field;
import frc.robot.subsystems.funnel.FunnelState;
import frc.robot.subsystems.funnel.FunnelStateHandler;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

	private final Robot robot;
	private final Swerve swerve;
	private final FunnelStateHandler funnelStateHandler;

	private RobotState currentState;

	public Superstructure(Robot robot) {
		this.robot = robot;
		this.swerve = robot.getSwerve();
		this.funnelStateHandler = new FunnelStateHandler(robot);
	}

	public RobotState getCurrentState() {
		return currentState;
	}

	public void logStatus() {
		Logger.recordOutput("CurrentState", currentState);
	}

	private boolean isNoteInShooter() {
		return robot.getFunnel().isNoteInShooter();
	}

	public Command setState(RobotState state) {
		return switch (state) {
			case IDLE -> idle();
			case INTAKE -> intake();
			case PRE_SPEAKER -> preSpeaker();
			case SPEAKER -> speaker();
			case PRE_AMP -> preAmp();
			case AMP -> amp();
			case TRANSFER_SHOOTER_ELEVATOR -> transferShooterElevator();
			case TRANSFER_ELEVATOR_SHOOTER -> transferElevatorShooter();
			case INTAKE_OUTTAKE -> intakeOuttake();
			case SHOOTER_OUTTAKE -> shooterOuttake();
		};
	}

	//@formatter:off
	public Command idle() {
		return new ParallelCommandGroup(
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
				funnelStateHandler.setState(FunnelState.STOP)
				//elevatorRoller.STOP
				//intakeRoller.STOP
				//flywheel.DEFAULT
				//intakePivot.IDLE
				//elevator.IDLE
		);
	}

	public Command intake() {
		return new ParallelCommandGroup(
				new SequentialCommandGroup(
						new ParallelCommandGroup(
								//intakePivot.INTAKE
								//intakeRoller.INTAKE
								funnelStateHandler.setState(FunnelState.STOP)
						).withTimeout(3), // .until(() -> isNoteInIntake
						new ParallelCommandGroup(
								funnelStateHandler.setState(FunnelState.NOTE_TO_SHOOTER)
								//intakePivot.IDLE
						).until(this::isNoteInShooter),
						new ParallelCommandGroup(
								//intakeRoller.STOP
								funnelStateHandler.setState(FunnelState.STOP)
						)
				),
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.NOTE))
				//flywheel.DEFAULT
				//elevator.IDLE
				//elevatorRoller.STOP
		);
	}

	public Command preSpeaker() {
		return new ParallelCommandGroup(
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER)),
				//flywheel.SHOOT
				funnelStateHandler.setState(FunnelState.STOP)
				//intakeRoller.STOP
				//elevator.IDLE
				//intakePivot.IDLE
				//elevatorRoller.STOP
		);
	}

	public Command speaker() {
		return new ParallelCommandGroup(
				new SequentialCommandGroup(
						funnelStateHandler.setState(FunnelState.STOP).withTimeout(3), // .until(() -> isReadyToShoot())
						funnelStateHandler.setState(FunnelState.SPEAKER).until(() -> !isNoteInShooter()),
						funnelStateHandler.setState(FunnelState.STOP)
				),
				//flywheel.SHOOT
				//intakeRoller.STOP
				//elevator.IDLE
				//intakePivot.IDLE
				//elevatorRoller.STOP
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER))
		);
	}

	public Command preAmp() {
		return new ParallelCommandGroup(
				new ParallelCommandGroup(
						swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP)),
						funnelStateHandler.setState(FunnelState.STOP)
						//elevator.PRE_SCORE
				).withTimeout(3) // .until(() -> isReadyToAmp())
				//intakeRoller.STOP
				//intakePivot.IDLE
				//flywheel.DEFAULT
		);
	}

	public Command amp() {
		return new ParallelCommandGroup(
				new SequentialCommandGroup(
						swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP))
								.until(() -> swerve.isAtHeading(Field.getAngleToAmp())),
						//elevator.SCORE.until(() -> isReadyToAmp())
						new ParallelCommandGroup(
								funnelStateHandler.setState(FunnelState.AMP)
								//elevatorRoller.SCORE
						), //.until(() -> !isNoteInElevator)
						new ParallelCommandGroup(
								funnelStateHandler.setState(FunnelState.STOP)
								//elevator.IDLE
								//elevatorRoller.STOP
						)
				)
				//intakeRoller.STOP
				//intakePivot.IDLE
				//flywheel.DEFAULT
		);
	}

	public Command transferShooterElevator() {
		return new ParallelCommandGroup(
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
				funnelStateHandler.setState(FunnelState.SHOOTER_TO_ELEVATOR)
				//intakeRoller.SHOOTER_TO_ELEVATOR
				//flywheel.DEFAULT
				//intakePivot.IDLE
				//elevator.IDLE
				//elevatorRoller.STOP
		);
	}

	public Command transferElevatorShooter() {
		return new ParallelCommandGroup(
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
				funnelStateHandler.setState(FunnelState.NOTE_TO_SHOOTER)
				//intakeRoller.ELEVATOR_TO_SHOOTER
				//flywheel.DEFAULT
				//intakePivot.IDLE
				//elevator.IDLE
				//elevatorRoller.STOP
		);
	}

	public Command intakeOuttake() {
		return new ParallelCommandGroup(
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
				funnelStateHandler.setState(FunnelState.INTAKE_OUTTAKE)
				//intakeRoller.OUTTAKE
				//intakePivot.INTAKE
				//flywheel.DEFAULT
				//elevator.IDLE
				//elevatorRoller.STOP
		); //.until(() -> !isNoteInIntake)
	}

	public Command shooterOuttake() {
		return new ParallelCommandGroup(
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
				funnelStateHandler.setState(FunnelState.SHOOTER_OUTTAKE)
				//intakeRoller.STOP
				//intakePivot.IDLE
				//flywheel.OUTTAKE
				//elevator.IDLE
				//elevatorRoller.STOP
		).until(() -> !isNoteInShooter());
	}
	//@formatter:on

}
