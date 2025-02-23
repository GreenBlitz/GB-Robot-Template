package frc.robot.statemachine.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.StateMachineConstants;
import frc.robot.statemachine.Tolerances;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmStateHandler;
import frc.robot.subsystems.climb.ClimbState;
import frc.robot.subsystems.climb.ClimbStateHandler;
import frc.robot.subsystems.climb.lifter.LifterStateHandler;
import frc.robot.subsystems.climb.solenoid.SolenoidStateHandler;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorStateHandler;
import frc.robot.subsystems.endeffector.EndEffectorState;
import frc.robot.subsystems.endeffector.EndEffectorStateHandler;
import org.littletonrobotics.junction.Logger;

import java.util.Set;

public class Superstructure extends GBSubsystem {

	private final Robot robot;
	private final ElevatorStateHandler elevatorStateHandler;
	private final ArmStateHandler armStateHandler;
	private final EndEffectorStateHandler endEffectorStateHandler;
	private final ClimbStateHandler climbStateHandler;

	private SuperstructureState currentState;
	public boolean driverIsCoralInOverride;
	public boolean driverIsAlgaeInOverride;

	public Superstructure(String logPath, Robot robot) {
		super(logPath);
		this.robot = robot;
		this.elevatorStateHandler = new ElevatorStateHandler(robot.getElevator());
		this.armStateHandler = new ArmStateHandler(robot.getArm());
		this.endEffectorStateHandler = new EndEffectorStateHandler(robot.getEndEffector(), this);
		this.climbStateHandler = new ClimbStateHandler(new SolenoidStateHandler(robot.getSolenoid()), new LifterStateHandler(robot.getLifter()));

		this.currentState = SuperstructureState.IDLE;
		this.driverIsCoralInOverride = false;
		this.driverIsAlgaeInOverride = false;
		setDefaultCommand(
			new DeferredCommand(() -> endState(currentState), Set.of(this, robot.getElevator(), robot.getArm(), robot.getEndEffector()))
		);
	}


	public Rotation2d getArmReversedSoftLimitByElevator() {
		return robot.getElevator().getElevatorPositionMeters() >= ArmConstants.ELEVATOR_HEIGHT_METERS_TO_CHANGE_SOFT_LIMIT
			? ArmConstants.ELEVATOR_OPEN_REVERSED_SOFTWARE_LIMIT
			: ArmConstants.ELEVATOR_CLOSED_REVERSED_SOFTWARE_LIMIT;
	}

	public boolean isCoralIn() {
		return robot.getEndEffector().isCoralIn() || driverIsCoralInOverride;
	}

	public boolean isAlgaeIn() {
		return robot.getEndEffector().isAlgaeIn() || driverIsAlgaeInOverride;
	}

	public boolean isClosed() {
		return robot.getElevator().isAtPosition(ElevatorState.CLOSED.getHeightMeters(), Tolerances.ELEVATOR_HEIGHT_METERS)
			&& elevatorStateHandler.getCurrentState() == ElevatorState.CLOSED
			&& robot.getArm().isAtPosition(ArmState.CLOSED.getPosition(), Tolerances.ARM_POSITION)
			&& armStateHandler.getCurrentState() == ArmState.CLOSED;
	}

	public boolean isPreScoreReady() {
		ScoreLevel targetScoreLevel = ScoringHelpers.targetScoreLevel;
		ArmState targetArmState = targetScoreLevel == ScoreLevel.L4 ? targetScoreLevel.getArmScore() : targetScoreLevel.getArmPreScore();

		return robot.getElevator().isAtPosition(targetScoreLevel.getElevatorPreScore().getHeightMeters(), Tolerances.ELEVATOR_HEIGHT_METERS)
			&& elevatorStateHandler.getCurrentState() == targetScoreLevel.getElevatorPreScore()
			&& robot.getArm().isAtPosition(targetArmState.getPosition(), Tolerances.ARM_POSITION)
			&& armStateHandler.getCurrentState() == targetArmState;
	}

	public boolean isReadyToScore() {
		ScoreLevel targetScoreLevel = ScoringHelpers.targetScoreLevel;
		return robot.getElevator().isAtPosition(targetScoreLevel.getElevatorScore().getHeightMeters(), Tolerances.ELEVATOR_HEIGHT_METERS)
			&& elevatorStateHandler.getCurrentState() == targetScoreLevel.getElevatorScore()
			&& robot.getArm().isAtPosition(targetScoreLevel.getArmScore().getPosition(), Tolerances.ARM_POSITION)
			&& armStateHandler.getCurrentState() == targetScoreLevel.getArmScore();
	}

	public boolean isReadyToOuttakeAlgae() {
		return robot.getElevator().isAtPosition(ElevatorState.PROCESSOR_OUTTAKE.getHeightMeters(), Tolerances.ELEVATOR_HEIGHT_METERS)
			&& elevatorStateHandler.getCurrentState() == ElevatorState.PROCESSOR_OUTTAKE
			&& robot.getArm().isAtPosition(ArmState.PROCESSOR_OUTTAKE.getPosition(), Tolerances.ARM_POSITION)
			&& armStateHandler.getCurrentState() == ArmState.PROCESSOR_OUTTAKE;
	}

	public boolean isReadyForNet() {
		return robot.getElevator().isAtPosition(ElevatorState.NET.getHeightMeters(), Tolerances.ELEVATOR_HEIGHT_METERS)
			&& elevatorStateHandler.getCurrentState() == ElevatorState.NET
			&& robot.getArm().isAtPosition(ArmState.NET.getPosition(), Tolerances.ARM_POSITION)
			&& armStateHandler.getCurrentState() == ArmState.NET;
	}

	@Override
	protected void subsystemPeriodic() {
		log();
	}

	private void log() {
		Logger.recordOutput(getLogPath() + "/ElevatorState", elevatorStateHandler.getCurrentState());
		Logger.recordOutput(getLogPath() + "/ArmState", armStateHandler.getCurrentState());
		Logger.recordOutput(getLogPath() + "/EndEffectorState", endEffectorStateHandler.getCurrentState());
	}

	public Command idle() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.CLOSED),
				armStateHandler.setState(ArmState.CLOSED),
				endEffectorStateHandler.setState(EndEffectorState.DEFAULT)
			),
			SuperstructureState.IDLE
		);
	}

	public Command stayInPlace() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.STAY_IN_PLACE),
				armStateHandler.setState(ArmState.STAY_IN_PLACE),
				endEffectorStateHandler.setState(EndEffectorState.DEFAULT)
			),
			SuperstructureState.STAY_IN_PLACE
		);
	}

	public Command intake() {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.INTAKE),
					armStateHandler.setState(ArmState.INTAKE),
					endEffectorStateHandler.setState(EndEffectorState.CORAL_INTAKE)
				).until(this::isCoralIn),
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.INTAKE),
					armStateHandler.setState(ArmState.INTAKE),
					endEffectorStateHandler.setState(EndEffectorState.CORAL_INTAKE)
				).withTimeout(StateMachineConstants.INTAKE_TIME_AFTER_BEAM_BREAK_SECONDS)
			),
			SuperstructureState.INTAKE
		);
	}

	public Command outtake() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.STAY_IN_PLACE),
				armStateHandler.setState(ArmState.STAY_IN_PLACE),
				endEffectorStateHandler.setState(EndEffectorState.CORAL_OUTTAKE)
			).until(() -> !isCoralIn()),
			SuperstructureState.OUTTAKE
		);
	}

	public Command armPreScore() {
		return asSubsystemCommand(
			new DeferredCommand(
				() -> new ParallelCommandGroup(
					elevatorStateHandler.setState(ScoringHelpers.targetScoreLevel.getElevatorWhileDrive()),
					armStateHandler.setState(ScoringHelpers.targetScoreLevel.getArmPreScore()),
					endEffectorStateHandler.setState(EndEffectorState.DEFAULT)
				),
				Set.of(this, robot.getElevator(), robot.getArm(), robot.getEndEffector())
			),
			SuperstructureState.ARM_PRE_SCORE
		);
	}

	private Command genericPreScore() {
		return asSubsystemCommand(
			new DeferredCommand(
				() -> new ParallelCommandGroup(
					elevatorStateHandler.setState(ScoringHelpers.targetScoreLevel.getElevatorPreScore()),
					armStateHandler.setState(ScoringHelpers.targetScoreLevel.getArmPreScore()),
					endEffectorStateHandler.setState(EndEffectorState.DEFAULT)
				),
				Set.of(this, robot.getElevator(), robot.getArm(), robot.getEndEffector())
			),
			SuperstructureState.PRE_SCORE
		);
	}

	private Command l4PreScore() {
		return asSubsystemCommand(
			new DeferredCommand(
				() -> new ParallelCommandGroup(
					new SequentialCommandGroup(
						armStateHandler.setState(ScoringHelpers.targetScoreLevel.getArmPreScore())
							.until(() -> robot.getElevator().isPastPosition(StateMachineConstants.ELEVATOR_POSITION_TO_MOVE_ARM_TO_SCORE_L4)),
						armStateHandler.setState(ScoringHelpers.targetScoreLevel.getArmScore())
					),
					elevatorStateHandler.setState(ScoringHelpers.targetScoreLevel.getElevatorPreScore()),
					endEffectorStateHandler.setState(EndEffectorState.DEFAULT)
				),
				Set.of(this, robot.getElevator(), robot.getArm(), robot.getEndEffector())
			),
			SuperstructureState.SCORE_WITHOUT_RELEASE
		);
	}

	public Command preScore() {
		return new DeferredCommand(() -> switch (ScoringHelpers.targetScoreLevel) {
			case L4 -> l4PreScore();
			case L1, L2, L3 -> genericPreScore();
		}, Set.of(this, robot.getElevator(), robot.getArm(), robot.getEndEffector()));
	}

	public Command scoreWithoutRelease() {
		return asSubsystemCommand(
			new DeferredCommand(
				() -> new ParallelCommandGroup(
					elevatorStateHandler.setState(ScoringHelpers.targetScoreLevel.getElevatorScore()),
					armStateHandler.setState(ScoringHelpers.targetScoreLevel.getArmScore()),
					endEffectorStateHandler.setState(EndEffectorState.DEFAULT)
				),
				Set.of(this, robot.getElevator(), robot.getArm(), robot.getEndEffector())
			),
			SuperstructureState.SCORE_WITHOUT_RELEASE
		);
	}

	public Command scoreWithRelease() {
		return asSubsystemCommand(
			new DeferredCommand(
				() -> new SequentialCommandGroup(
					new ParallelCommandGroup(
						elevatorStateHandler.setState(ScoringHelpers.targetScoreLevel.getElevatorScore()),
						armStateHandler.setState(ScoringHelpers.targetScoreLevel.getArmScore()),
						endEffectorStateHandler.setState(ScoringHelpers.targetScoreLevel.getEndEffectorScore())
					).until(() -> !isCoralIn()),
					new ParallelCommandGroup(
						elevatorStateHandler.setState(ScoringHelpers.targetScoreLevel.getElevatorScore()),
						armStateHandler.setState(ScoringHelpers.targetScoreLevel.getArmScore()),
						endEffectorStateHandler.setState(ScoringHelpers.targetScoreLevel.getEndEffectorScore())
					).withTimeout(StateMachineConstants.SCORE_OUTTAKE_TIME_AFTER_BEAM_BREAK_SECONDS)
				),
				Set.of(this, robot.getElevator(), robot.getArm(), robot.getEndEffector())
			),
			SuperstructureState.SCORE
		);
	}

	public Command closeL4AfterScore() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					armStateHandler.setState(ArmState.MID_WAY_CLOSE)
						.until(() -> !robot.getElevator().isPastPosition(StateMachineConstants.ELEVATOR_POSITION_TO_CLOSE_ARM)),
					armStateHandler.setState(ArmState.CLOSED)
				),
				new SequentialCommandGroup(
					elevatorStateHandler.setState(ElevatorState.PRE_L4)
						.until(() -> robot.getArm().isPastPosition(StateMachineConstants.ARM_POSITION_TO_CLOSE_ELEVATOR_L4)),
					elevatorStateHandler.setState(ElevatorState.CLOSED)
				),
				endEffectorStateHandler.setState(EndEffectorState.DEFAULT)
			).until(this::isClosed),
			SuperstructureState.CLOSE_L4
		);
	}

	public Command afterScore() {
		return new DeferredCommand(
			() -> ScoringHelpers.targetScoreLevel == ScoreLevel.L4 ? closeL4AfterScore() : preScore(),
			Set.of(this, robot.getElevator(), robot.getArm(), robot.getEndEffector())
		);
	}

	public Command algaeRemove() {
		return asSubsystemCommand(
			new DeferredCommand(
				() -> new SequentialCommandGroup(
					new ParallelCommandGroup(
						elevatorStateHandler.setState(ScoringHelpers.getAlgaeRemoveLevel().getElevatorState()),
						armStateHandler.setState(ScoringHelpers.getAlgaeRemoveLevel().getArmState()),
						endEffectorStateHandler.setState(ScoringHelpers.getAlgaeRemoveLevel().getEndEffectorState())
					)
//							.until(this::isAlgaeIn),
//					new ParallelCommandGroup(
//						elevatorStateHandler.setState(ScoringHelpers.getAlgaeRemoveLevel().getElevatorState()),
//						armStateHandler.setState(ScoringHelpers.getAlgaeRemoveLevel().getArmState()),
//						endEffectorStateHandler.setState(ScoringHelpers.getAlgaeRemoveLevel().getEndEffectorState())
//					).withTimeout(StateMachineConstants.ALGAE_REMOVE_TIME_AFTER_LIMIT_SWITCH_SECONDS)
				),
				Set.of(robot.getElevator(), robot.getArm(), robot.getEndEffector(), this)
			),
			SuperstructureState.ALGAE_REMOVE
		);
	}

	public Command postAlgaeRemove() {
		return asSubsystemCommand(
			new DeferredCommand(
				() -> new ParallelCommandGroup(
					elevatorStateHandler.setState(ScoringHelpers.getAlgaeRemoveLevel().getPostElevatorState()),
					armStateHandler.setState(ScoringHelpers.getAlgaeRemoveLevel().getPostArmState()),
					endEffectorStateHandler.setState(EndEffectorState.DEFAULT)
				),
				Set.of(robot.getElevator(), robot.getArm(), robot.getEndEffector(), this)
			),
			SuperstructureState.POST_ALGAE_REMOVE
		);
	}

	public Command processorScore() {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.PROCESSOR_OUTTAKE),
					armStateHandler.setState(ArmState.PROCESSOR_OUTTAKE),
					endEffectorStateHandler.setState(EndEffectorState.DEFAULT)
				).until(this::isReadyToOuttakeAlgae),
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.PROCESSOR_OUTTAKE),
					armStateHandler.setState(ArmState.PROCESSOR_OUTTAKE),
					endEffectorStateHandler.setState(EndEffectorState.PROCESSOR_OUTTAKE)
				)
			),
//					.until(() -> !isAlgaeIn()),
			SuperstructureState.PROCESSOR_OUTTAKE
		);
	}

	public Command algaeOuttake() {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.ALGAE_OUTTAKE),
					armStateHandler.setState(ArmState.ALGAE_OUTTAKE),
					endEffectorStateHandler.setState(EndEffectorState.DEFAULT)
				).until(this::isReadyToOuttakeAlgae),
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.ALGAE_OUTTAKE),
					armStateHandler.setState(ArmState.ALGAE_OUTTAKE),
					endEffectorStateHandler.setState(EndEffectorState.ALGAE_OUTTAKE)
				)
			),
			// .until(() -> !isAlgaeIn()),
			SuperstructureState.ALGAE_OUTTAKE
		);
	}

	public Command preNet() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.WHILE_DRIVE_NET),
				armStateHandler.setState(ArmState.PRE_NET),
				endEffectorStateHandler.setState(EndEffectorState.DEFAULT)
			),
			SuperstructureState.PRE_NET
		);
	}


	public Command netWithoutRelease() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.NET),
				armStateHandler.setState(ArmState.NET),
				endEffectorStateHandler.setState(EndEffectorState.DEFAULT)
			),
			SuperstructureState.NET_WITHOUT_RELEASE
		);
	}


	public Command netWithRelease() {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.NET),
					armStateHandler.setState(ArmState.NET),
					endEffectorStateHandler.setState(EndEffectorState.NET_OUTTAKE)
				)
//						.until(() -> !isAlgaeIn()),
//				new ParallelCommandGroup(
//					elevatorStateHandler.setState(ElevatorState.NET),
//					armStateHandler.setState(ArmState.NET),
//					endEffectorStateHandler.setState(EndEffectorState.NET_OUTTAKE)
//				).withTimeout(StateMachineConstants.NET_OUTTAKE_TIME_AFTER_LIMIT_SWITCH_SECONDS)
			),
			SuperstructureState.NET_WITH_RELEASE
		);
	}


	public Command preClimb() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					new ParallelCommandGroup(armStateHandler.setState(ArmState.CLIMB), climbStateHandler.setState(ClimbState.STOP))
						.until(() -> robot.getArm().isPastPosition(StateMachineConstants.ARM_POSITION_TO_DEPLOY_LIFTER)),
					new ParallelCommandGroup(climbStateHandler.setState(ClimbState.DEPLOY), armStateHandler.setState(ArmState.CLIMB))
				),
				elevatorStateHandler.setState(ElevatorState.CLOSED),
				endEffectorStateHandler.setState(EndEffectorState.STOP)
			),
			SuperstructureState.PRE_CLIMB
		);
	}

	public Command climb() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.CLOSED),
				armStateHandler.setState(ArmState.CLIMB),
				endEffectorStateHandler.setState(EndEffectorState.STOP),
				climbStateHandler.setState(ClimbState.CLIMB)
			),
			SuperstructureState.CLIMB
		);
	}

	public Command climbStop() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.CLOSED),
				armStateHandler.setState(ArmState.CLIMB),
				endEffectorStateHandler.setState(EndEffectorState.STOP),
				climbStateHandler.setState(ClimbState.STOP)
			),
			SuperstructureState.CLIMB_STOP
		);
	}

	private Command asSubsystemCommand(Command command, SuperstructureState state) {
		return new ParallelCommandGroup(asSubsystemCommand(command, state.name()), new InstantCommand(() -> currentState = state));
	}

	private Command endState(SuperstructureState state) {
		return switch (state) {
			case STAY_IN_PLACE, OUTTAKE -> stayInPlace();
			case INTAKE, IDLE, POST_ALGAE_REMOVE, ALGAE_OUTTAKE, CLOSE_L4, PROCESSOR_OUTTAKE -> idle();
			case ALGAE_REMOVE -> postAlgaeRemove();
			case ARM_PRE_SCORE -> armPreScore();
			case PRE_SCORE, SCORE, SCORE_WITHOUT_RELEASE -> afterScore();
			case PRE_NET, NET_WITHOUT_RELEASE, NET_WITH_RELEASE -> preNet();
			case PRE_CLIMB -> preClimb();
			case CLIMB, CLIMB_STOP -> climbStop();
		};
	}

}
