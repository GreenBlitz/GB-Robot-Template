package frc.robot.subsystems.elevatorRoller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.DoubleSupplier;

public class ElevatorRollerCommandsBuilder {

	private final ElevatorRoller elevatorRoller;

	public ElevatorRollerCommandsBuilder(ElevatorRoller elevatorRoller) {
		this.elevatorRoller = elevatorRoller;
	}

	//@formatter:off
	public Command setPower(double power){
		return new FunctionalCommand(
				()-> {},
				()-> elevatorRoller.setPower(power),
				interrupted-> elevatorRoller.stop(),
				()-> false,
				elevatorRoller
		).withName("Set power to " + power);
	}

	public Command setPower(DoubleSupplier power){
		return new FunctionalCommand(
				()-> {},
				()-> elevatorRoller.setPower(power.getAsDouble()),
				interrupted-> elevatorRoller.stop(),
				()-> false,
				elevatorRoller
		).withName("Set power by supplier");
	}

	public Command stop() {
		return new RunCommand(elevatorRoller::stop, elevatorRoller).withName("Stop");
	}
	//@formatter:on

}
