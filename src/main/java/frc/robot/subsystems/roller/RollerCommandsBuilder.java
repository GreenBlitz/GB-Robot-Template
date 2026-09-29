package frc.robot.subsystems.roller;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.command2.Command;
import org.wpilib.command2.DeferredCommand;
import org.wpilib.command2.RunCommand;
import frc.utils.utilcommands.InitExecuteCommand;

import java.util.Set;
import java.util.function.Supplier;

public class RollerCommandsBuilder {

	private final Roller motor;

	protected RollerCommandsBuilder(Roller motor) {
		this.motor = motor;
		motor.setDefaultCommand(stop());
	}

	public Command setVoltage(double voltage) {
		return motor.asSubsystemCommand(new RunCommand(() -> motor.setVoltage(voltage)), "Set motor voltage to " + voltage);
	}

	public Command setVoltage(Supplier<Double> voltage) {
		return motor.asSubsystemCommand(new RunCommand(() -> motor.setVoltage(voltage.get())), "Set motor voltage");
	}

	public Command stop() {
		return motor.asSubsystemCommand(new RunCommand(motor::stop), "Stop motor");
	}

	public Command setPower(Supplier<Double> supplier) {
		return motor.asSubsystemCommand(new RunCommand(() -> motor.setPower(supplier.get())), "Set power with supplier");
	}

	public Command setPower(Double power) {
		return motor.asSubsystemCommand(new RunCommand(() -> motor.setPower(power)), "Set power to " + power);
	}

	public Command rollRotationsAtVoltageForwards(double rotations, double voltage) {
		double finalVoltage = Math.abs(voltage);
		return motor.asSubsystemCommand(
			new DeferredCommand(
				() -> new InitExecuteCommand(
					() -> motor.updateTargetPosition(Rotation2d.fromRotations(rotations + motor.getPosition().getRotations())),
					() -> motor.setVoltage(finalVoltage)
				).until(motor::isPastTargetPosition),
				Set.of(motor)
			),
			"Roll " + rotations + " rotations"
		);
	}

	public Command rollRotationsAtVoltageBackwards(double rotations, double voltage) {
		double finalVoltage = -Math.abs(voltage);
		return motor.asSubsystemCommand(
			new DeferredCommand(
				() -> new InitExecuteCommand(
					() -> motor.updateTargetPosition(Rotation2d.fromRotations(motor.getPosition().getRotations() - rotations)),
					() -> motor.setVoltage(finalVoltage)
				).until(motor::isBehindTargetPosition),
				Set.of(motor)
			),
			"Roll " + rotations + " rotations backwards"
		);
	}

}

