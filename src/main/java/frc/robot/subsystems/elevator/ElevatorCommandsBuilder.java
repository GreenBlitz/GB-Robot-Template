package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
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

	public Command setTargetPosition(Rotation2d angle) {
		return new FunctionalCommand(
			() -> {},
			() -> elevator.setTargetPosition(angle),
			interrupted -> elevator.stayInPlace(),
			() -> false,
			elevator
		);
	}

	public Command stop() {
		return new RunCommand(elevator::stop, elevator).withName("Stop");
	}

	public Command calibrateFeedForward() {
		return new ParallelCommandGroup(
			new LoggedDashboardCommand("ks", elevator::setVoltage, elevator).withName("Ks calibration"),
			new LoggedDashboardCommand("kg", elevator::setVoltage, elevator).withName("Kg calibration")
		);
	}
	//@formatter:on

}
