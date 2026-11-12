package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.utils.utilcommands.ExecuteEndCommand;
import frc.utils.utilcommands.InitExecuteCommand;

import java.util.function.Supplier;

public class DynamicMotionMagicElevatorCommandsBuilder extends ElevatorCommandBuilder {

	private final DynamicMotionMagicElevator elevator;

	public DynamicMotionMagicElevatorCommandsBuilder(DynamicMotionMagicElevator elevator) {
		super(elevator);
		this.elevator = elevator;
	}


	@Override
	public Command setTargetPosition(double target) {
		return elevator.asSubsystemCommand(
			new InitExecuteCommand(() -> elevator.setTargetPosition(target), () -> {}, elevator),
			"move to position: " + target
		);
	}

	@Override
	public Command setTargetPosition(Supplier<Double> target) {
		return elevator.asSubsystemCommand(
			new RunCommand(() -> elevator.setTargetPosition(target.get()), elevator),
			"move with supplier to position: " + target
		);
	}

	@Override
	public Command setTargetPosition(Rotation2d target) {
		return elevator.asSubsystemCommand(
			new InitExecuteCommand(() -> elevator.setTargetPosition(target), () -> {}, elevator),
			"move to position: " + target
		);
	}

	@Override
	public Command setTargetPositionStopAtLimit(Double target) {
		return elevator.asSubsystemCommand(
			new ExecuteEndCommand(() -> elevator.setTargetPosition(target), elevator::stayInPlace),
			"moves to a target but stops moving if hits limit, target: " + target
		);
	}

	@Override
	public Command setTargetPositionStopAtLimit(Rotation2d target) {
		return elevator.asSubsystemCommand(
			new ExecuteEndCommand(() -> elevator.setTargetPosition(target), elevator::stayInPlace),
			"moves to a target but stops moving if hits limit, target: " + target
		);
	}

	@Override
	public Command setTargetPositionStopAtLimit(Supplier<Double> target) {
		return elevator.asSubsystemCommand(
			new SequentialCommandGroup(setTargetPosition(target).until(() -> !elevator.isInSoftLimit()), stayPut()),
			"moves to a target but stops moving if hits limit, target: " + target
		);
	}

	public Command setTargetPosition(Rotation2d target, double maxVelocityRPS, double maxAccelerationRPSSquared) {
		return elevator.asSubsystemCommand(
			new InitExecuteCommand(() -> elevator.setTargetPosition(target, maxVelocityRPS, maxAccelerationRPSSquared), () -> {}, elevator),
			"move to position: " + target
		);
	}

	public Command setTargetPosition(double target, double maxVelocityRPS, double maxAccelerationRPSSquared) {
		return elevator.asSubsystemCommand(
			new InitExecuteCommand(() -> elevator.setTargetPosition(target, maxVelocityRPS, maxAccelerationRPSSquared), () -> {}, elevator),
			"move to position: " + target
		);
	}

	public Command setTargetPositionStopAtLimit(double target, double maxVelocityRPS, double maxAccelerationRPSSquared) {
		return elevator.asSubsystemCommand(
			new InitExecuteCommand(() -> elevator.setTargetPosition(target, maxVelocityRPS, maxAccelerationRPSSquared), () -> {}, elevator),
			"move to position: " + target
		);
	}

	public Command setTargetPositionStopAtLimit(Rotation2d target, double maxVelocityRPS, double maxAccelerationRPSSquared) {
		return elevator.asSubsystemCommand(
			new InitExecuteCommand(() -> elevator.setTargetPosition(target, maxVelocityRPS, maxAccelerationRPSSquared), () -> {}, elevator),
			"move to position: " + target
		);
	}

}
