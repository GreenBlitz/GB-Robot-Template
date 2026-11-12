package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.utils.utilcommands.ExecuteEndCommand;
import frc.utils.utilcommands.InitExecuteCommand;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ElevatorCommandBuilder {

	public final Elevator elevator;

	public ElevatorCommandBuilder(Elevator elevator) {
		this.elevator = elevator;
		this.elevator.setDefaultCommand(stayPut());
	}

	public Command stop() {
		return elevator.asSubsystemCommand(new RunCommand(elevator::stop, elevator), "stop elevator");
	}

	public Command stayPut() {
		return elevator.asSubsystemCommand(new RunCommand(elevator::stayInPlace, elevator), "stay in place, dont move");
	}

	public Command setTargetPosition(double target) {
		return elevator.asSubsystemCommand(
			new InitExecuteCommand(() -> elevator.setTargetPosition(target), () -> {}, elevator),
			"move to position: " + target
		);
	}


	public Command setTargetPosition(Supplier<Double> target) {
		return elevator.asSubsystemCommand(
			new RunCommand(() -> elevator.setTargetPosition(target.get()), elevator),
			"move with supplier to position: " + target
		);
	}

	public Command setTargetPosition(Rotation2d target) {
		return elevator.asSubsystemCommand(
			new InitExecuteCommand(() -> elevator.setTargetPosition(target), () -> {}, elevator),
			"move to position: " + target
		);
	}

	public Command setTargetPositionStopAtLimit(Double target) {
		return elevator.asSubsystemCommand(
			new ExecuteEndCommand(() -> elevator.setTargetPosition(target), elevator::stayInPlace),
			"moves to a target but stops moving if hits limit, target: " + target
		);
	}

	public Command setTargetPositionStopAtLimit(Rotation2d target) {
		return elevator.asSubsystemCommand(
			new ExecuteEndCommand(() -> elevator.setTargetPosition(target), elevator::stayInPlace),
			"moves to a target but stops moving if hits limit, target: " + target
		);
	}

	public Command setTargetPositionStopAtLimit(Supplier<Double> target) {
		return elevator.asSubsystemCommand(
			new SequentialCommandGroup(setTargetPosition(target).until(() -> !elevator.isInSoftLimit()), stayPut()),
			"moves to a target but stops moving if hits limit, target: " + target
		);
	}

	public Command setVoltage(double voltage) {
		return elevator
			.asSubsystemCommand(new InitExecuteCommand(() -> elevator.setVoltage(voltage), () -> {}, elevator), "set voltage to: " + voltage);
	}

	public Command setVoltage(Supplier<Double> voltage) {
		return elevator.asSubsystemCommand(
			new RunCommand(() -> elevator.setTargetPosition(voltage.get()), elevator),
			"seti with supplier voltage to : " + voltage
		);
	}

	public Command setPower(double power) {
		return elevator.asSubsystemCommand(new RunCommand(() -> elevator.setPower(power)), "Set power to: " + power);
	}

	public Command setPower(DoubleSupplier powerSupplier) {
		return elevator.asSubsystemCommand(new RunCommand(() -> elevator.setPower(powerSupplier.getAsDouble())), "Set power by supplier");
	}

	public Command setVoltageWithoutLimit(double voltage, BooleanSupplier isFinished) {
		return elevator
			.asSubsystemCommand(
				new ExecuteEndCommand(() -> elevator.setVoltageWithoutLimit(voltage), elevator::stayInPlace),
				"Set voltage to: " + voltage + " without limits"
			)
			.until(isFinished);
	}

}
