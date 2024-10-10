package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.constants.Field;
import frc.robot.subsystems.elevatorRoller.ElevatorRollerState;
import frc.robot.subsystems.elevatorRoller.ElevatorRollerStateHandler;
import frc.robot.subsystems.flywheel.FlywheelState;
import frc.robot.subsystems.flywheel.FlywheelStateHandler;
import frc.robot.subsystems.funnel.FunnelState;
import frc.robot.subsystems.funnel.FunnelStateHandler;
import frc.robot.subsystems.intake.pivot.PivotState;
import frc.robot.subsystems.intake.pivot.PivotStateHandler;
import frc.robot.subsystems.intake.roller.IntakeStates;
import frc.robot.subsystems.intake.roller.IntakeStatesHandler;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

	private final Robot robot;
	private final Swerve swerve;
	private final ElevatorRollerStateHandler elevatorRollerStateHandler;
	private final FlywheelStateHandler flywheelStateHandler;
	private final FunnelStateHandler funnelStateHandler;
	private final IntakeStatesHandler intakeStatesHandler;
	private final PivotStateHandler pivotStateHandler;

	private RobotState currentState;

	public Superstructure(Robot robot) {
		this.robot = robot;
		this.swerve = robot.getSwerve();
		this.elevatorRollerStateHandler = new ElevatorRollerStateHandler(robot);
		this.flywheelStateHandler = new FlywheelStateHandler(robot);
		this.funnelStateHandler = new FunnelStateHandler(robot);
		this.intakeStatesHandler = new IntakeStatesHandler(robot.getIntakeRoller());
		this.pivotStateHandler = new PivotStateHandler(robot.getPivot());
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

	private boolean isNoteInElevatorRoller() {
		return robot.getElevatorRoller().isNoteIn();
	}

	private boolean isNoteInIntake() {
		return robot.getIntakeRoller().isNoteIn();
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
			funnelStateHandler.setState(FunnelState.STOP),
			elevatorRollerStateHandler.setState(ElevatorRollerState.STOP),
			intakeStatesHandler.setState(IntakeStates.STOP),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			pivotStateHandler.setState(PivotState.UP)
			//elevator.IDLE
		);
	}

	public Command intake() {
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					pivotStateHandler.setState(PivotState.DOWN),
					intakeStatesHandler.setState(IntakeStates.INTAKE),
					funnelStateHandler.setState(FunnelState.STOP)
				).until(this::isNoteInIntake),
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.NOTE_TO_SHOOTER),
					pivotStateHandler.setState(PivotState.UP)
				).until(this::isNoteInShooter),
				new ParallelCommandGroup(
					intakeStatesHandler.setState(IntakeStates.STOP),
					funnelStateHandler.setState(FunnelState.STOP)
				)
			),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.NOTE)),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			//elevator.IDLE
			elevatorRollerStateHandler.setState(ElevatorRollerState.STOP),
			funnelStateHandler.setState(FunnelState.NOTE_TO_SHOOTER),
			intakeStatesHandler.setState(IntakeStates.INTAKE)
		);
	}

	public Command preSpeaker() {
		return new ParallelCommandGroup(
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER)),
			flywheelStateHandler.setState(FlywheelState.SHOOTING),
			funnelStateHandler.setState(FunnelState.STOP),
			intakeStatesHandler.setState(IntakeStates.STOP),
			//elevator.IDLE
			pivotStateHandler.setState(PivotState.UP),
			elevatorRollerStateHandler.setState(ElevatorRollerState.STOP)
		);
	}

	public Command speaker() {
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				funnelStateHandler.setState(FunnelState.STOP).withTimeout(3), // .until(() -> isReadyToShoot())
				funnelStateHandler.setState(FunnelState.SPEAKER).until(() -> !isNoteInShooter()),
				funnelStateHandler.setState(FunnelState.STOP)
			),
			flywheelStateHandler.setState(FlywheelState.SHOOTING),
			intakeStatesHandler.setState(IntakeStates.STOP),
			//elevator.IDLE
			pivotStateHandler.setState(PivotState.UP),
			elevatorRollerStateHandler.setState(ElevatorRollerState.STOP),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER))
		);
	}

	public Command preAmp() {
		return new ParallelCommandGroup(
			new ParallelCommandGroup(
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP)),
				funnelStateHandler.setState(FunnelState.STOP)
				//elevator.PRE_SCORE
			).withTimeout(3), // .until(() -> isReadyToAmp())
			intakeStatesHandler.setState(IntakeStates.STOP),
			pivotStateHandler.setState(PivotState.UP),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			elevatorRollerStateHandler.setState(ElevatorRollerState.TRANSFER_TO_ELEVATOR)
		).until(this::isNoteInElevatorRoller);
	}

	public Command amp() {
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP),
					//elevator.IDLE
					elevatorRollerStateHandler.setState(ElevatorRollerState.STOP)
				),
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP))
				.until(() -> swerve.isAtHeading(Field.getAngleToAmp())),
				//elevator.SCORE.until(() -> isReadyToAmp())
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.AMP),
					elevatorRollerStateHandler.setState(ElevatorRollerState.AMP)
				).until(() -> !isNoteInElevatorRoller()),
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP),
					//elevator.IDLE
					elevatorRollerStateHandler.setState(ElevatorRollerState.STOP)
				)
			),
			intakeStatesHandler.setState(IntakeStates.STOP),
			pivotStateHandler.setState(PivotState.UP),
			flywheelStateHandler.setState(FlywheelState.DEFAULT)
		);
	}

	public Command transferShooterElevator() {
		return new ParallelCommandGroup(
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
			funnelStateHandler.setState(FunnelState.SHOOTER_TO_ELEVATOR),
			intakeStatesHandler.setState(IntakeStates.NOTE_TO_SHOOTER),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			pivotStateHandler.setState(PivotState.UP),
			//elevator.IDLE
			elevatorRollerStateHandler.setState(ElevatorRollerState.TRANSFER_TO_ELEVATOR)
		);
	}

	public Command transferElevatorShooter() {
		return new ParallelCommandGroup(
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
			funnelStateHandler.setState(FunnelState.NOTE_TO_SHOOTER),
			intakeStatesHandler.setState(IntakeStates.NOTE_TO_SHOOTER),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			pivotStateHandler.setState(PivotState.UP),
			//elevator.IDLE
			elevatorRollerStateHandler.setState(ElevatorRollerState.TRANSFER_FROM_ELEVATOR)
		);
	}

	public Command intakeOuttake() {
		return new ParallelCommandGroup(
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
			funnelStateHandler.setState(FunnelState.INTAKE_OUTTAKE),
			intakeStatesHandler.setState(IntakeStates.OUTTAKE),
			pivotStateHandler.setState(PivotState.DOWN),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			//elevator.IDLE
			elevatorRollerStateHandler.setState(ElevatorRollerState.TRANSFER_FROM_ELEVATOR)
		).until(() -> !isNoteInIntake());
	}

	public Command shooterOuttake() {
		return new ParallelCommandGroup(
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
			funnelStateHandler.setState(FunnelState.SHOOTER_OUTTAKE),
			intakeStatesHandler.setState(IntakeStates.STOP),
			pivotStateHandler.setState(PivotState.UP),
			flywheelStateHandler.setState(FlywheelState.SHOOTER_OUTTAKE),
			//elevator.IDLE
			elevatorRollerStateHandler.setState(ElevatorRollerState.STOP)
		).until(() -> !isNoteInShooter());
	}
	//@formatter:on

}
