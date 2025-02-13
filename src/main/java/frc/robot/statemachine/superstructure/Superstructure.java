package frc.robot.statemachine.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.statemachine.StateMachineConstants;
import frc.robot.statemachine.Tolerances;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmStateHandler;
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

	private SuperstructureState currentState;

	public Superstructure(String logPath, Robot robot) {
		super(logPath);
		this.robot = robot;
		this.elevatorStateHandler = new ElevatorStateHandler(robot.getElevator());
		this.armStateHandler = new ArmStateHandler(robot.getArm());
		this.endEffectorStateHandler = new EndEffectorStateHandler(robot.getEndEffector());

		this.currentState = SuperstructureState.IDLE;
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
		return robot.getEndEffector().isCoralInBack();
	}

	public boolean isCoralOut() {
		return !robot.getEndEffector().isCoralInFront();
	}

	public boolean isPreScoreReady(ScoreLevel scoreLevel) {
		return robot.getElevator().isAtPosition(scoreLevel.getElevatorPreScore().getHeightMeters(), Tolerances.ELEVATOR_HEIGHT_METERS)
			&& elevatorStateHandler.getCurrentState() == scoreLevel.getElevatorPreScore()
			&& robot.getArm().isAtPosition(scoreLevel.getArmPreScore().getPosition(), Tolerances.ARM_POSITION)
			&& armStateHandler.getCurrentState() == scoreLevel.getArmPreScore();
	}

	public boolean isReadyToScore(ScoreLevel scoreLevel) {
		return robot.getElevator().isAtPosition(scoreLevel.getElevatorScore().getHeightMeters(), Tolerances.ELEVATOR_HEIGHT_METERS)
			&& elevatorStateHandler.getCurrentState() == scoreLevel.getElevatorScore()
			&& robot.getArm().isAtPosition(scoreLevel.getArmScore().getPosition(), Tolerances.ARM_POSITION)
			&& armStateHandler.getCurrentState() == scoreLevel.getArmScore();
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

	public Command intake() {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.INTAKE),
					armStateHandler.setState(ArmState.INTAKE),
					endEffectorStateHandler.setState(EndEffectorState.INTAKE)
				).until(this::isCoralIn),
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.INTAKE),
					armStateHandler.setState(ArmState.INTAKE),
					endEffectorStateHandler.setState(EndEffectorState.INTAKE)
				).withTimeout(StateMachineConstants.INTAKE_TIME_AFTER_BEAM_BREAK_SECONDS)
			),
			SuperstructureState.INTAKE
		);
	}

	public Command outtake() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.OUTTAKE),
				armStateHandler.setState(ArmState.OUTTAKE),
				endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
			).until(this::isCoralOut),
			SuperstructureState.OUTTAKE
		);
	}

	public Command genericArmPreScore(ScoreLevel scoreLevel) {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.CLOSED),
				armStateHandler.setState(scoreLevel.getArmPreScore()),
				endEffectorStateHandler.setState(EndEffectorState.DEFAULT)
			),
			scoreLevel.getSuperstructureArmPreScore()
		);
	}

	public Command genericPreScore(ScoreLevel scoreLevel) {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(scoreLevel.getElevatorPreScore()),
				armStateHandler.setState(scoreLevel.getArmPreScore()),
				endEffectorStateHandler.setState(EndEffectorState.DEFAULT)
			),
			scoreLevel.getSuperstructurePreScore()
		);
	}

	public Command genericScoreWithoutRelease(ScoreLevel scoreLevel) {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(scoreLevel.getElevatorScore()),
				armStateHandler.setState(scoreLevel.getArmScore()),
				endEffectorStateHandler.setState(EndEffectorState.DEFAULT)
			),
			scoreLevel.getSuperstructureScoreWithoutRelease()
		);
	}

	public Command genericScoreWithRelease(ScoreLevel scoreLevel) {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					elevatorStateHandler.setState(scoreLevel.getElevatorScore()),
					armStateHandler.setState(scoreLevel.getArmScore()),
					endEffectorStateHandler.setState(scoreLevel.getEndEffectorScore())
				).until(this::isCoralOut),
				new ParallelCommandGroup(
					elevatorStateHandler.setState(scoreLevel.getElevatorScore()),
					armStateHandler.setState(scoreLevel.getArmScore()),
					endEffectorStateHandler.setState(scoreLevel.getEndEffectorScore())
				).withTimeout(StateMachineConstants.SCORE_OUTTAKE_TIME_AFTER_BEAM_BREAK_SECONDS)
			),
			scoreLevel.getSuperstructureScore()
		);
	}

	private Command asSubsystemCommand(Command command, SuperstructureState state) {
		return new ParallelCommandGroup(asSubsystemCommand(command, state.name()), new InstantCommand(() -> currentState = state));
	}

	private Command endState(SuperstructureState state) {
		return switch (state) {
			case INTAKE, OUTTAKE, IDLE -> idle();
			case ARM_PRE_L1 -> genericArmPreScore(ScoreLevel.L1);
			case ARM_PRE_L2 -> genericArmPreScore(ScoreLevel.L2);
			case ARM_PRE_L3 -> genericArmPreScore(ScoreLevel.L3);
			case ARM_PRE_L4 -> genericArmPreScore(ScoreLevel.L4);
			case PRE_L1, SCORE_L1, SCORE_L1_WITHOUT_RELEASE -> genericPreScore(ScoreLevel.L1);
			case PRE_L2, SCORE_L2, SCORE_L2_WITHOUT_RELEASE -> genericPreScore(ScoreLevel.L2);
			case PRE_L3, SCORE_L3, SCORE_L3_WITHOUT_RELEASE -> genericPreScore(ScoreLevel.L3);
			case PRE_L4, SCORE_L4, SCORE_L4_WITHOUT_RELEASE -> genericPreScore(ScoreLevel.L4);
		};
	}

}
