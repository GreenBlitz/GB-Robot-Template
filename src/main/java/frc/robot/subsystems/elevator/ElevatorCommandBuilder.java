package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.DoubleSupplier;

public class ElevatorCommandBuilder {

	private final Elevator elevator;

	public ElevatorCommandBuilder(Elevator elevator) {
		this.elevator = elevator;
	}

	//@formatter:off
    public Command setPower(double power) {
        return new FunctionalCommand(
                () -> elevator.setPower(power),
                () -> {},
                interrupted -> elevator.stop(),
                () -> false,
                elevator
        ).withName("Set power: " + power);
    }

    public Command setPower(DoubleSupplier power) {
        return new FunctionalCommand(
                () -> elevator.setPower(power.getAsDouble()),
                () -> {},
                interrupted -> elevator.stop(),
                () -> false,
                elevator
        ).withName("Set power: " + power);
    }

    public Command setPosition(Rotation2d angle) {
        return new InstantCommand(() -> elevator.setTargetAngle(angle));
    }
    //@formatter:on

}
