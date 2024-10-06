package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.elevator.factories.RealElevatorConstants;
import frc.utils.utilcommands.LoggedDashboardCommand;

import java.util.function.DoubleSupplier;

public class ElevatorCommandBuilder {

	private final Elevator elevator;

	public ElevatorCommandBuilder(Elevator elevator) {
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
        ).withName("Set power: " + power);
    }

    public Command setTargetPosition(Rotation2d angle) {
        return new InstantCommand(() -> elevator.setTargetAngle(angle));
    }

    public Command stayInPlace() {
        return new FunctionalCommand(
                () -> {},
                elevator::stayInPlace,
                interrupted -> elevator.stop(),
                () -> false,
                elevator
        );
    }

    public Command stop() {
        return new InstantCommand(elevator::stop);
    }

    public static Command calibratedFeedForward() {
        double[] oldValues = new double[] {RealElevatorConstants.FEEDFORWARD_CALCULATOR.ka, RealElevatorConstants.FEEDFORWARD_CALCULATOR.kg, RealElevatorConstants.FEEDFORWARD_CALCULATOR.kv, RealElevatorConstants.FEEDFORWARD_CALCULATOR.ks};
        return new ParallelCommandGroup(
                new LoggedDashboardCommand("ks", (Double newkS) -> RealElevatorConstants.FEEDFORWARD_CALCULATOR = new ElevatorFeedforward(oldValues[0], oldValues[1], oldValues[2], newkS)),
                new LoggedDashboardCommand("kg", (Double newkG) -> RealElevatorConstants.FEEDFORWARD_CALCULATOR = new ElevatorFeedforward(oldValues[0], newkG, oldValues[2], oldValues[3]))
        );
    }
    //@formatter:on

}
