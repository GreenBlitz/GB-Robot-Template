package frc.robot.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.JoysticksBindings;
import frc.robot.Robot;
import frc.robot.constants.Field;
import frc.robot.subsystems.elevator.ElevatorStates;
import frc.robot.subsystems.elevator.ElevatorStatesHandler;
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
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import frc.utils.joysticks.SmartJoystick;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

	private final String logPath;

	private final Robot robot;
	private final Swerve swerve;
	private final ElevatorRollerStateHandler elevatorRollerStateHandler;
	private final FlywheelStateHandler flywheelStateHandler;
	private final FunnelStateHandler funnelStateHandler;
	private final IntakeStatesHandler intakeStatesHandler;
	private final PivotStateHandler pivotStateHandler;
	public final ElevatorStatesHandler elevatorStatesHandler;

	private RobotState currentState;

	public Superstructure(String logPath, Robot robot) {
		this.logPath = logPath;

		this.robot = robot;
		this.swerve = robot.getSwerve();
		this.elevatorRollerStateHandler = new ElevatorRollerStateHandler(robot.getElevatorRoller());
		this.flywheelStateHandler = new FlywheelStateHandler(robot.getFlywheel());
		this.funnelStateHandler = new FunnelStateHandler(robot.getFunnel());
		this.intakeStatesHandler = new IntakeStatesHandler(robot.getIntakeRoller());
		this.pivotStateHandler = new PivotStateHandler(robot.getPivot());
		this.elevatorStatesHandler = new ElevatorStatesHandler(robot.getElevator());
	}

	public RobotState getCurrentState() {
		return currentState;
	}

	public void logStatus() {
		Logger.recordOutput(logPath + "CurrentState", currentState);
	}

	private boolean isNoteInShooter() {
		return robot.getFunnel().isNoteInShooter();
	}

	private boolean isNoteInElevatorRoller() {
		return robot.getElevatorRoller().isNoteIn();
	}

	private boolean isReadyToShoot() {
		boolean isFlywheelReady = robot.getFlywheel().isAtVelocity(FlywheelState.SHOOTING.getVelocity(), Tolerances.FLYWHEEL_VELOCITY_TOLERANCE);
		boolean isSwerveReady = swerve.isAtHeading(
			SwerveMath.getRelativeTranslation(
				robot.getPoseEstimator().getEstimatedPose().getTranslation(),
				Field.getSpeaker().toTranslation2d()).getAngle()
		);
		return isFlywheelReady && isSwerveReady;
	}

	private Command noteInRumble() {
		SmartJoystick mainJoystick = JoysticksBindings.getMainJoystick();
		SmartJoystick secondJoystick = JoysticksBindings.getSecondJoystick();
		return new FunctionalCommand(
			() -> {},
			() -> {
				mainJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.7);
				secondJoystick.setRumble(GenericHID.RumbleType.kBothRumble, 0.7);
			},
			interrupted -> {
				mainJoystick.stopRumble(GenericHID.RumbleType.kBothRumble);
				secondJoystick.stopRumble(GenericHID.RumbleType.kBothRumble);
			},
			() -> false
		).withTimeout(0.5);
	}

	private boolean isReadyToAmp() {
		boolean isElevatorReady = MathUtil.isNear(
			ElevatorStates.AMP.getPositionMeters(),
			robot.getElevator().getPositionMeters(),
			Tolerances.ELEVATOR_POSITION_METERS_TOLERANCE
		);
		// boolean isSwerveReady = swerve.isAtHeading(amp);
		return isElevatorReady;// && isSwerveReady
	}

	private Command setCurrentStateName(RobotState state) {
		return new InstantCommand(() -> currentState = state);
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
			setCurrentStateName(RobotState.IDLE),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
			elevatorRollerStateHandler.setState(ElevatorRollerState.STOP),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			funnelStateHandler.setState(FunnelState.STOP),
			intakeStatesHandler.setState(IntakeStates.STOP),
			pivotStateHandler.setState(PivotState.UP),
			elevatorStatesHandler.setState(ElevatorStates.IDLE)
		);
	}

	public Command intake() {
		return new ParallelCommandGroup(
			setCurrentStateName(RobotState.INTAKE),
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.NOTE_TO_SHOOTER),
					intakeStatesHandler.setState(IntakeStates.INTAKE),
					pivotStateHandler.setState(PivotState.ON_FLOOR)
				).until(this::isNoteInShooter),
				new ParallelCommandGroup(
					noteInRumble(),
					pivotStateHandler.setState(PivotState.UP),
					funnelStateHandler.setState(FunnelState.STOP),
					intakeStatesHandler.setState(IntakeStates.STOP)
				)
			),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.NOTE)),
			elevatorRollerStateHandler.setState(ElevatorRollerState.STOP),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			elevatorStatesHandler.setState(ElevatorStates.IDLE)
		);
	}

	public Command preSpeaker() {
		return new ParallelCommandGroup(
			setCurrentStateName(RobotState.PRE_SPEAKER),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER)),
			elevatorRollerStateHandler.setState(ElevatorRollerState.STOP),
			flywheelStateHandler.setState(FlywheelState.SHOOTING),
			funnelStateHandler.setState(FunnelState.STOP),
			intakeStatesHandler.setState(IntakeStates.STOP),
			pivotStateHandler.setState(PivotState.UP),
			elevatorStatesHandler.setState(ElevatorStates.IDLE)
		);
	}

	public Command speaker() {
		return new ParallelCommandGroup(
			setCurrentStateName(RobotState.SPEAKER),
			new SequentialCommandGroup(
				funnelStateHandler.setState(FunnelState.STOP).until(this::isReadyToShoot),
				funnelStateHandler.setState(FunnelState.SPEAKER).until(() -> !isNoteInShooter()),
				funnelStateHandler.setState(FunnelState.STOP)
			),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER)),
			elevatorRollerStateHandler.setState(ElevatorRollerState.STOP),
			flywheelStateHandler.setState(FlywheelState.SHOOTING),
			intakeStatesHandler.setState(IntakeStates.STOP),
			pivotStateHandler.setState(PivotState.UP),
			elevatorStatesHandler.setState(ElevatorStates.IDLE)
		);
	}

	public Command preAmp() {
		return new ParallelCommandGroup(
			setCurrentStateName(RobotState.PRE_AMP),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP)),
			funnelStateHandler.setState(FunnelState.STOP),
			elevatorRollerStateHandler.setState(ElevatorRollerState.STOP),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			intakeStatesHandler.setState(IntakeStates.STOP),
			pivotStateHandler.setState(PivotState.UP),
			elevatorStatesHandler.setState(ElevatorStates.IDLE)
		);
	}

	public Command amp() {
		return new ParallelCommandGroup(
			setCurrentStateName(RobotState.AMP),
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP)),
					elevatorRollerStateHandler.setState(ElevatorRollerState.STOP),
					elevatorStatesHandler.setState(ElevatorStates.AMP)
				).until(this::isReadyToAmp),
				new ParallelCommandGroup(
					swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP)),
					elevatorRollerStateHandler.setState(ElevatorRollerState.AMP),
					elevatorStatesHandler.setState(ElevatorStates.AMP)
				).until(() -> !isNoteInElevatorRoller()),//.withTimeout(Timeouts.AMP_SECONDS),
				new ParallelCommandGroup(
					swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP)),
					elevatorRollerStateHandler.setState(ElevatorRollerState.AMP),
					elevatorStatesHandler.setState(ElevatorStates.AMP)
				).withTimeout(Timeouts.AMP_AFTER_SENSOR_SECONDS),
				new ParallelCommandGroup(
					swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
					elevatorRollerStateHandler.setState(ElevatorRollerState.STOP),
					elevatorStatesHandler.setState(ElevatorStates.IDLE)
				)
			),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			intakeStatesHandler.setState(IntakeStates.STOP),
			pivotStateHandler.setState(PivotState.UP),
			funnelStateHandler.setState(FunnelState.STOP)
		);
	}

	public Command transferShooterElevator() {
		return new ParallelCommandGroup(
			setCurrentStateName(RobotState.TRANSFER_SHOOTER_ELEVATOR),
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					intakeStatesHandler.setState(IntakeStates.INTAKE),
					funnelStateHandler.setState(FunnelState.SHOOTER_TO_ELEVATOR),
					elevatorRollerStateHandler.setState(ElevatorRollerState.TRANSFER_TO_ELEVATOR)
				).until(this::isNoteInElevatorRoller),//.withTimeout(Timeouts.AMP_SECONDS),
				new ParallelCommandGroup(
					intakeStatesHandler.setState(IntakeStates.STOP),
					funnelStateHandler.setState(FunnelState.STOP),
					elevatorRollerStateHandler.setState(ElevatorRollerState.STOP)
				)
			),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			elevatorStatesHandler.setState(ElevatorStates.IDLE),
			pivotStateHandler.setState(PivotState.UP)
		);
	}

	public Command transferElevatorShooter() {
		return new ParallelCommandGroup(
			setCurrentStateName(RobotState.TRANSFER_ELEVATOR_SHOOTER),
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					intakeStatesHandler.setState(IntakeStates.SHOOTER_TO_ELEVATOR),
					elevatorRollerStateHandler.setState(ElevatorRollerState.TRANSFER_FROM_ELEVATOR),
					funnelStateHandler.setState(FunnelState.NOTE_TO_SHOOTER)
				).until(this::isNoteInShooter),
				new ParallelCommandGroup(
					intakeStatesHandler.setState(IntakeStates.STOP),
					elevatorRollerStateHandler.setState(ElevatorRollerState.STOP),
					funnelStateHandler.setState(FunnelState.STOP)
				)
			),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			elevatorStatesHandler.setState(ElevatorStates.IDLE),
			pivotStateHandler.setState(PivotState.UP)
		);
	}

	public Command intakeOuttake() {
		return new ParallelCommandGroup(
			setCurrentStateName(RobotState.INTAKE_OUTTAKE),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
			elevatorRollerStateHandler.setState(ElevatorRollerState.TRANSFER_FROM_ELEVATOR),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			funnelStateHandler.setState(FunnelState.INTAKE_OUTTAKE),
			intakeStatesHandler.setState(IntakeStates.OUTTAKE),
			pivotStateHandler.setState(PivotState.UP),
			elevatorStatesHandler.setState(ElevatorStates.IDLE)
		);
	}

	public Command shooterOuttake() {
		return new ParallelCommandGroup(
			setCurrentStateName(RobotState.SHOOTER_OUTTAKE),
			new SequentialCommandGroup(
				funnelStateHandler.setState(FunnelState.SHOOTER_OUTTAKE).until(() -> !isNoteInShooter()),
				funnelStateHandler.setState(FunnelState.STOP)
			),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
			elevatorRollerStateHandler.setState(ElevatorRollerState.STOP),
			flywheelStateHandler.setState(FlywheelState.SHOOTER_OUTTAKE),
			intakeStatesHandler.setState(IntakeStates.STOP),
			pivotStateHandler.setState(PivotState.UP),
			elevatorStatesHandler.setState(ElevatorStates.IDLE)
		);
	}
	//@formatter:on

}
