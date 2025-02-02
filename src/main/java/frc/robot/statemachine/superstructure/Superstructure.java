package frc.robot.statemachine.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.statemachine.Tolerances;
import frc.robot.subsystems.GBSubsystem;
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
		setDefaultCommand(new DeferredCommand(() -> endState(currentState), Set.of(this)));
	}

	public boolean isCoralIn() {
		return robot.getEndEffector().isCoralInBack();
	}

	public boolean isCoralOut() {
		return !robot.getEndEffector().isCoralInFront();
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
				endEffectorStateHandler.setState(EndEffectorState.KEEP)
			),
			SuperstructureState.IDLE
		);
	}

	public Command intake() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.INTAKE),
				armStateHandler.setState(ArmState.INTAKE),
				endEffectorStateHandler.setState(EndEffectorState.INTAKE)
			).until(this::isCoralIn),
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

	private Command preScoreUtil(ScoreLevel scoreLevel) {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(scoreLevel.getElevatorPreScore()),
				armStateHandler.setState(scoreLevel.getArmPreScore()),
				endEffectorStateHandler.setState(EndEffectorState.KEEP)
			),
			scoreLevel.getSuperstructurePreScore()
		);
	}

	private Command preL1() {
		return preScoreUtil(ScoreLevel.L1);
	}

	private Command preL2() {
		return preScoreUtil(ScoreLevel.L2);
	}

	private Command preL3() {
		return preScoreUtil(ScoreLevel.L3);
	}

	private Command preL4() {
		return preScoreUtil(ScoreLevel.L4);
	}

	public Command preScore(ScoreLevel scoreLevel) {
		return switch (scoreLevel) {
			case L1 -> preL1();
			case L2 -> preL2();
			case L3 -> preL3();
			case L4 -> preL4();
		};
	}

	private Command scoreUtil(ScoreLevel scoreLevel) {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					elevatorStateHandler.setState(scoreLevel.getElevatorScore()),
					armStateHandler.setState(scoreLevel.getArmScore()),
					endEffectorStateHandler.setState(EndEffectorState.KEEP)
				).until(() -> isReadyToScore(scoreLevel)),
				new ParallelCommandGroup(
					elevatorStateHandler.setState(scoreLevel.getElevatorScore()),
					armStateHandler.setState(scoreLevel.getArmScore()),
					endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
				)
			).until(this::isCoralOut),
			scoreLevel.getSuperstructureScore()
		);
	}

	private Command scoreL1() {
		return scoreUtil(ScoreLevel.L1);
	}

	private Command scoreL2() {
		return scoreUtil(ScoreLevel.L2);
	}

	private Command scoreL3() {
		return scoreUtil(ScoreLevel.L3);
	}

	private Command scoreL4() {
		return scoreUtil(ScoreLevel.L4);
	}

	public Command score(ScoreLevel scoreLevel) {
		return switch (scoreLevel) {
			case L1 -> scoreL1();
			case L2 -> scoreL2();
			case L3 -> scoreL3();
			case L4 -> scoreL4();
		};
	}

	private Command asSubsystemCommand(Command command, SuperstructureState state) {
		return new ParallelCommandGroup(asSubsystemCommand(command, state.name()), new InstantCommand(() -> currentState = state));
	}

	private Command endState(SuperstructureState state) {
		return switch (state) {
			case INTAKE, OUTTAKE, IDLE -> idle();
			case PRE_L1, SCORE_L1 -> preL1();
			case PRE_L2, SCORE_L2 -> preL2();
			case PRE_L3, SCORE_L3 -> preL3();
			case PRE_L4, SCORE_L4 -> preL4();
		};
	}

}
