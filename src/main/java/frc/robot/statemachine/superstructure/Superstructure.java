package frc.robot.statemachine.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
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

public class Superstructure extends GBSubsystem {

	private final Robot robot;
	private final ElevatorStateHandler elevatorStateHandler;
	private final ArmStateHandler armStateHandler;
	private final EndEffectorStateHandler endEffectorStateHandler;

	public Superstructure(String logPath, Robot robot) {
		super(logPath);
		this.robot = robot;
		this.elevatorStateHandler = new ElevatorStateHandler(robot.getElevator());
		this.armStateHandler = new ArmStateHandler(robot.getArm());
		this.endEffectorStateHandler = new EndEffectorStateHandler(robot.getEndEffector());
	}

	public boolean isCoralIn() {
		return robot.getEndEffector().isCoralInBack();
	}

	public boolean isCoralOut() {
		return !robot.getEndEffector().isCoralInFront();
	}

	public boolean isReadyToScore(ScoreLevel scoreLevel) {
		return robot.getElevator().isAtPosition(scoreLevel.getElevatorPositionMeters(), Tolerances.ELEVATOR_HEIGHT_METERS)
			&& robot.getArm().isAtPosition(scoreLevel.getArmPosition(), Tolerances.ARM_POSITION);
	}

	public Command idle() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.CLOSED),
				armStateHandler.setState(ArmState.CLOSED),
				endEffectorStateHandler.setState(EndEffectorState.KEEP)
			),
			"idle"
		);
	}

	public Command intake() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.FEEDER),
				armStateHandler.setState(ArmState.INTAKE),
				endEffectorStateHandler.setState(EndEffectorState.INTAKE)
			).until(this::isCoralIn),
			"intake"
		);
	}

	public Command outtake() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.OUTTAKE),
				armStateHandler.setState(ArmState.OUTTAKE),
				endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
			).until(this::isCoralOut),
			"outtake"
		);
	}

	public Command preL1() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.L1),
				armStateHandler.setState(ArmState.PRE_L1),
				endEffectorStateHandler.setState(EndEffectorState.KEEP)
			),
			"pre l1"
		);
	}

	public Command preL2() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.L2),
				armStateHandler.setState(ArmState.PRE_L2),
				endEffectorStateHandler.setState(EndEffectorState.KEEP)
			),
			"pre l2"
		);
	}

	public Command preL3() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.L3),
				armStateHandler.setState(ArmState.PRE_L3),
				endEffectorStateHandler.setState(EndEffectorState.KEEP)
			),
			"pre l3"
		);
	}

	public Command preL4() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.L4),
				armStateHandler.setState(ArmState.PRE_L4),
				endEffectorStateHandler.setState(EndEffectorState.KEEP)
			),
			"pre l4"
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
			"score l1"
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
			"score l2"
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
			"score l3"
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
			"score l4"
		);
	}

}
