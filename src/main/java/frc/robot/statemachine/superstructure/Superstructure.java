package frc.robot.statemachine.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
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

	private Command genericPreScore(ScoreLevel scoreLevel) {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(scoreLevel.getElevatorPreScore()),
				armStateHandler.setState(scoreLevel.getArmPreScore()),
				endEffectorStateHandler.setState(EndEffectorState.KEEP)
			),
			scoreLevel.getSuperstructurePreScore()
		);
	}

	public Command preL1() {
		return genericPreScore(ScoreLevel.L1);
	}

	public Command preL2() {
		return genericPreScore(ScoreLevel.L2);
	}

	public Command preL3() {
		return genericPreScore(ScoreLevel.L3);
	}

	public Command preL4() {
		return genericPreScore(ScoreLevel.L4);
	}

	public Command preScore(ScoreLevel scoreLevel) {
		return switch (scoreLevel) {
			case L1 -> preL1();
			case L2 -> preL2();
			case L3 -> preL3();
			case L4 -> preL4();
		};
	}

	private Command genericScore(ScoreLevel scoreLevel) {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					elevatorStateHandler.setState(scoreLevel.getElevatorPreScore()),
					armStateHandler.setState(scoreLevel.getArmPreScore()),
					endEffectorStateHandler.setState(EndEffectorState.KEEP)
				).until(() -> isPreScoreReady(scoreLevel)),
				new ParallelCommandGroup(
					elevatorStateHandler.setState(scoreLevel.getElevatorScore()),
					armStateHandler.setState(scoreLevel.getArmScore()),
					endEffectorStateHandler.setState(EndEffectorState.KEEP)
				).until(() -> isReadyToScore(scoreLevel)),
				new ParallelCommandGroup(
					elevatorStateHandler.setState(scoreLevel.getElevatorScore()),
					armStateHandler.setState(scoreLevel.getArmScore()),
					endEffectorStateHandler.setState(scoreLevel.getEndEffectorScore())
				)
			).until(this::isCoralOut),
			scoreLevel.getSuperstructureScore()
		);
	}

	public Command scoreL1() {
		return genericScore(ScoreLevel.L1);
	}

	public Command scoreL2() {
		return genericScore(ScoreLevel.L2);
	}

	public Command scoreL3() {
		return genericScore(ScoreLevel.L3);
	}

	public Command scoreL4() {
		return genericScore(ScoreLevel.L4);
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
