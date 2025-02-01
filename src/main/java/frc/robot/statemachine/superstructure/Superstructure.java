package frc.robot.statemachine.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.statemachine.RobotState;
import frc.robot.statemachine.Tolerances;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmStateHandler;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorStateHandler;
import frc.robot.subsystems.endeffector.EndEffectorState;
import frc.robot.subsystems.endeffector.EndEffectorStateHandler;

public class Superstructure {

	private final String logPath;
	private final Robot robot;
	private final ElevatorStateHandler elevatorStateHandler;
	private final ArmStateHandler armStateHandler;
	private final EndEffectorStateHandler endEffectorStateHandler;

	public Superstructure(String logPath, Robot robot) {
		this.logPath = logPath;
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
			RobotState.DRIVE
		);
	}

	public Command intake() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.FEEDER),
				armStateHandler.setState(ArmState.INTAKE),
				endEffectorStateHandler.setState(EndEffectorState.INTAKE)
			).until(this::isCoralIn),
			RobotState.INTAKE
		);
	}

	public Command l1() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					new RunCommand(() -> {}).until(() -> isReadyToScore(ScoreLevel.L1)),
					endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
				),
				elevatorStateHandler.setState(ElevatorState.L1),
				armStateHandler.setState(ArmState.L1)
			).until(this::isCoralOut),
			RobotState.L1
		);
	}

	public Command l2() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					new RunCommand(() -> {}).until(() -> isReadyToScore(ScoreLevel.L2)),
					endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
				),
				elevatorStateHandler.setState(ElevatorState.L2),
				armStateHandler.setState(ArmState.L2)
			).until(this::isCoralOut),
			RobotState.L2
		);
	}

	public Command l3() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					new RunCommand(() -> {}).until(() -> isReadyToScore(ScoreLevel.L3)),
					endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
				),
				elevatorStateHandler.setState(ElevatorState.L3),
				armStateHandler.setState(ArmState.L3)
			).until(this::isCoralOut),
			RobotState.L3
		);
	}

	public Command l4() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				new SequentialCommandGroup(
					new RunCommand(() -> {}).until(() -> isReadyToScore(ScoreLevel.L4)),
					endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
				),
				elevatorStateHandler.setState(ElevatorState.L4),
				armStateHandler.setState(ArmState.L4)
			).until(this::isCoralOut),
			RobotState.L4
		);
	}

	public Command preL1() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.L1),
				armStateHandler.setState(ArmState.PRE_L1),
				endEffectorStateHandler.setState(EndEffectorState.KEEP)
			),
			RobotState.PRE_L1
		);
	}

	public Command preL2() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.L2),
				armStateHandler.setState(ArmState.PRE_L2),
				endEffectorStateHandler.setState(EndEffectorState.KEEP)
			),
			RobotState.PRE_L2
		);
	}

	public Command preL3() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.L3),
				armStateHandler.setState(ArmState.PRE_L3),
				endEffectorStateHandler.setState(EndEffectorState.KEEP)
			),
			RobotState.PRE_L3
		);
	}

	public Command preL4() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.L4),
				armStateHandler.setState(ArmState.PRE_L4),
				endEffectorStateHandler.setState(EndEffectorState.KEEP)
			),
			RobotState.PRE_L4
		);
	}

	public Command outtake() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.OUTTAKE),
				armStateHandler.setState(ArmState.OUTTAKE),
				endEffectorStateHandler.setState(EndEffectorState.OUTTAKE)
			).until(this::isCoralOut),
			RobotState.OUTTAKE
		);
	}

	public Command alignReef() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				elevatorStateHandler.setState(ElevatorState.CLOSED),
				armStateHandler.setState(ArmState.CLOSED),
				endEffectorStateHandler.setState(EndEffectorState.KEEP)
			),
			RobotState.ALIGN_REEF
		);
	}

	public Command asSubsystemCommand(Command command, RobotState state) {
		return new InstantCommand();
//		command = super.asSubsystemCommand(command, state.name());
//		return new ParallelCommandGroup(new InstantCommand(() -> currentState = state), command);
	}

}
