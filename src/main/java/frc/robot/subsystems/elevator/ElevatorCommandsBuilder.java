package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.*;
import frc.utils.utilcommands.LoggedDashboardCommand;

import java.util.function.DoubleSupplier;

public class ElevatorCommandsBuilder {

	private final Elevator elevator;

	public ElevatorCommandsBuilder(Elevator elevator) {
		this.elevator = elevator;
	}

	//@formatter:off
	public Command setPower(double power) {
		return new FunctionalCommand(
			() -> {},
			() -> elevator.setPower(power),
			interrupted -> elevator.stop(),
			() -> false,
			elevator
		).withName("Set power: " + power);
	}

	public Command setPower(DoubleSupplier power) {
		return new FunctionalCommand(
			() -> {},
			() -> elevator.setPower(power.getAsDouble()),
			interrupted -> elevator.stop(),
			() -> false,
			elevator
		).withName("Set power by supplier");
	}

	public Command setTargetPositionMeters(double position) {
		return new FunctionalCommand(
			() -> {},
			() -> elevator.setTargetPositionMeters(position),
			interrupted -> elevator.stayInPlace(),
			() -> false,
			elevator
		);
	}

	public Command stop() {
		return new RunCommand(elevator::stop, elevator).withName("Stop");
	}

	public Command calibrateFeedForward() {
		return new LoggedDashboardCommand("StaticFeedForward", elevator::setVoltage, elevator).withName("Static Feed-Forward calibration");
	}
	//@formatter:on

}
