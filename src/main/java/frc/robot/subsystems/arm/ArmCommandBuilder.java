package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.GBCommandsBuilder;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ArmCommandBuilder extends GBCommandsBuilder {

	private final Arm arm;

	public ArmCommandBuilder(Arm arm) {
		this.arm = arm;
	}

	public Command stayInPlace() {
		return arm.asSubsystemCommand(new RunCommand(arm::stayInPlace, arm), "Stay in place");
	}

	public Command setPower(double power) {
		return arm.asSubsystemCommand(new RunCommand(() -> arm.setPower(power), arm), "Set power to: " + power);
	}

	public Command setPower(DoubleSupplier powerSupplier) {
		return arm.asSubsystemCommand(new RunCommand(() -> arm.setPower(powerSupplier.getAsDouble())), "Set power by supplier");
	}

	public Command setNeutralMode(boolean brake) {
		return arm.asSubsystemCommand(new InstantCommand(() -> arm.setBrake(brake), arm), "Set neutral mode to: " + brake);
	}

	public Command setTargetPosition(Rotation2d target) {
		return arm.asSubsystemCommand(new RunCommand(() -> arm.setTargetPosition(target), arm), "Set target position to: " + target);
	}

	public Command setTargetPosition(Supplier<Rotation2d> target) {
		return arm.asSubsystemCommand(new RunCommand(() -> arm.setTargetPosition(target.get()), arm), "Set target with Supplier");
	}

	public Command setVoltage(Double voltage) {
		return arm.asSubsystemCommand(new RunCommand(() -> arm.setVoltage(voltage), arm), "Set voltage to: " + voltage);
	}

	public Command setVoltage(DoubleSupplier voltage) {
		return arm.asSubsystemCommand(new RunCommand(() -> arm.setVoltage(voltage.getAsDouble()), arm), "Set voltage by supplier");
	}

}

