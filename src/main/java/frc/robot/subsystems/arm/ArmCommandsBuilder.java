package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.utils.utilcommands.ExecuteEndCommand;
import frc.utils.utilcommands.LoggedDashboardCommand;

import java.util.function.DoubleSupplier;

public class ArmCommandsBuilder {

	private final Arm arm;

	public ArmCommandsBuilder(Arm arm) {
		this.arm = arm;
	}

	public Command stop() {
		return arm.asSubsystemCommand(new RunCommand(arm::stop), "Stop");
	}

	public Command setPower(double power) {
		return arm.asSubsystemCommand(new RunCommand(() -> arm.setPower(power)), "Set power to: " + power);
	}

	public Command setPower(DoubleSupplier powerSupplier) {
		return arm.asSubsystemCommand(new RunCommand(() -> arm.setPower(powerSupplier.getAsDouble())), "Set power by supplier");
	}

	public Command setVoltage(double voltage) {
		return arm.asSubsystemCommand(new RunCommand(() -> arm.setVoltage(voltage)), "Set voltage to: " + voltage);
	}

	public Command moveToPosition(Rotation2d position) {
		return arm
			.asSubsystemCommand(new ExecuteEndCommand(() -> arm.setTargetPosition(position), arm::stop), "Set target position to: " + position);
	}

	public Command stayInPlace() {
		return arm
			.asSubsystemCommand(new FunctionalCommand(arm::stayInPlace, () -> {}, interrupted -> arm.stop(), () -> false), "Stay in place");
	}

	public Command loggedDashboardSetVoltage() {
		return arm.asSubsystemCommand(new LoggedDashboardCommand("Set Arm Voltage", arm::setVoltage), "Set voltage via dashboard");
	}

}
