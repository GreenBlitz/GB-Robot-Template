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
		return robot.getElevator().isAtPosition(scoreLevel.getElevatorState().getHeightMeters(), Tolerances.ELEVATOR_HEIGHT_METERS)
			&& elevatorStateHandler.getCurrentState() == scoreLevel.getElevatorState()
			&& robot.getArm().isAtPosition(scoreLevel.getArmState().getPosition(), Tolerances.ARM_POSITION)
			&& armStateHandler.getCurrentState() == scoreLevel.getArmState();
	}

	@Override
	protected void subsystemPeriodic() {
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

	public Command preL1() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.L1),
				armStateHandler.setState(ArmState.PRE_L1),
				endEffectorStateHandler.setState(EndEffectorState.KEEP)
			),
			SuperstructureState.PRE_L1
		);
	}

	public Command preL2() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.L2),
				armStateHandler.setState(ArmState.PRE_L2),
				endEffectorStateHandler.setState(EndEffectorState.KEEP)
			),
			SuperstructureState.PRE_L2
		);
	}

	public Command preL3() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.L3),
				armStateHandler.setState(ArmState.PRE_L3),
				endEffectorStateHandler.setState(EndEffectorState.KEEP)
			),
			SuperstructureState.PRE_L3
		);
	}

	public Command preL4() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.L4),
				armStateHandler.setState(ArmState.PRE_L4),
				endEffectorStateHandler.setState(EndEffectorState.KEEP)
			),
			SuperstructureState.PRE_L4
		);
	}

	public Command scoreL1() {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.L1),
					armStateHandler.setState(ArmState.L1),
					endEffectorStateHandler.setState(EndEffectorState.KEEP)
				).until(() -> isReadyToScore(ScoreLevel.L1)),
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.L1),
					armStateHandler.setState(ArmState.L1),
					endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
				)
			).until(this::isCoralOut),
			SuperstructureState.SCORE_L1
		);
	}

	public Command scoreL2() {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.L2),
					armStateHandler.setState(ArmState.L2),
					endEffectorStateHandler.setState(EndEffectorState.KEEP)
				).until(() -> isReadyToScore(ScoreLevel.L2)),
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.L2),
					armStateHandler.setState(ArmState.L2),
					endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
				)
			).until(this::isCoralOut),
			SuperstructureState.SCORE_L2
		);
	}

	public Command scoreL3() {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.L3),
					armStateHandler.setState(ArmState.L3),
					endEffectorStateHandler.setState(EndEffectorState.KEEP)
				).until(() -> isReadyToScore(ScoreLevel.L3)),
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.L3),
					armStateHandler.setState(ArmState.L3),
					endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
				)
			).until(this::isCoralOut),
			SuperstructureState.SCORE_L3
		);
	}

	public Command scoreL4() {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.L4),
					armStateHandler.setState(ArmState.L4),
					endEffectorStateHandler.setState(EndEffectorState.KEEP)
				).until(() -> isReadyToScore(ScoreLevel.L4)),
				new ParallelCommandGroup(
					elevatorStateHandler.setState(ElevatorState.L4),
					armStateHandler.setState(ArmState.L4),
					endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
				)
			).until(this::isCoralOut),
			SuperstructureState.SCORE_L4
		);
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
