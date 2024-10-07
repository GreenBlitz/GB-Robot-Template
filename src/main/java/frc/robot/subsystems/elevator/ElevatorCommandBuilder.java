package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
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
        ).withName("Set power by supplier with value: " + power.getAsDouble());
    }

    public Command setTargetPosition(Rotation2d angle) {
        return new FunctionalCommand(
            () -> {},
            () -> elevator.setTargetAngle(angle),
            interrupted -> elevator.stayInPlace(),
            () -> false,
            elevator
        );
    }

    public Command stop() {
        return new RunCommand(elevator::stop);
    }

    public Command calibrateFeedForward() {
        double[] oldValues = new double[] {RealElevatorConstants.FEEDFORWARD_CALCULATOR.ka, RealElevatorConstants.FEEDFORWARD_CALCULATOR.kg, RealElevatorConstants.FEEDFORWARD_CALCULATOR.kv, RealElevatorConstants.FEEDFORWARD_CALCULATOR.ks};
        return new ParallelCommandGroup(
            new LoggedDashboardCommand("ks", elevator::setPower, elevator),
            new LoggedDashboardCommand("kg", elevator::setPower, elevator)
        );
    }
    //@formatter:on

}
