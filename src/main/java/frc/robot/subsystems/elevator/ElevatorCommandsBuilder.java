package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.*;
import frc.utils.utilcommands.ExecuteEndCommand;
import frc.utils.utilcommands.InitExecuteCommand;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ElevatorCommandsBuilder {

	private final Elevator elevator;

	public ElevatorCommandsBuilder(Elevator elevator) {
		this.elevator = elevator;
	}

	public Command stop() {
		return elevator.asSubsystemCommand(new RunCommand(elevator::stop, elevator), "stop elevator");
	}

	public Command stayInPlace() {
		return elevator.asSubsystemCommand(new RunCommand(elevator::stayInPlace, elevator), "stay in place");
	}

	public Command setTargetPosition(double positionMeters) {
		return elevator.asSubsystemCommand(
			new InitExecuteCommand(() -> elevator.setTargetPosition(positionMeters), () -> {}, elevator),
			"move to position in meters: " + positionMeters
		);
	}


	public Command setTargetPosition(Supplier<Double> positionMeters) {
		return elevator.asSubsystemCommand(
			new RunCommand(() -> elevator.setTargetPosition(positionMeters.get()), elevator),
			"move with supplier to position in meters: " + positionMeters
		);
	}

	public Command setTargetPositionStopAtLimit(Double positionMeters) {
		return elevator.asSubsystemCommand(
			new ExecuteEndCommand(() -> elevator.setTargetPosition(positionMeters), elevator::stayInPlace,elevator),
			"moves to a positionMeters but stops moving if hits limit, positionMeters: " + positionMeters
		);
	}

	public Command setTargetPositionStopAtLimit(Supplier<Double> positionMeters) {
		return elevator.asSubsystemCommand(
			new SequentialCommandGroup(setTargetPosition(positionMeters).until(() -> !elevator.isInMechanicalLimit()), stayInPlace()),
			"moves to a positionMeters but stops moving if hits limit, positionMeters: " + positionMeters
		);
	}

	public Command setVoltage(double voltage) {
		return elevator
			.asSubsystemCommand(new InitExecuteCommand(() -> elevator.setVoltage(voltage), () -> {}, elevator), "set voltage to: " + voltage);
	}

	public Command setVoltage(Supplier<Double> voltage) {
		return elevator.asSubsystemCommand(
			new RunCommand(() -> elevator.setTargetPosition(voltage.get()), elevator),
			"set with supplier voltage to : " + voltage.get()
		);
	}

	public Command setPower(double power) {
		return elevator.asSubsystemCommand(new RunCommand(() -> elevator.setPower(power)), "Set power to: " + power);
	}

	public Command setPower(Supplier<Double> power) {
		return elevator.asSubsystemCommand(new RunCommand(() -> elevator.setPower(power.get())), "Set power by supplier");
	}

	public Command setVoltageWithoutLimit(double voltage, BooleanSupplier isFinished) {
		return elevator
			.asSubsystemCommand(
				new ExecuteEndCommand(() -> elevator.setVoltageWithoutLimit(voltage), elevator::stayInPlace,elevator),
				"Set voltage to: " + voltage + " without limits"
			)
			.until(isFinished);
	}

}
