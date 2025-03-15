package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.utils.utilcommands.InitExecuteCommand;
import frc.utils.utilcommands.LoggedDashboardCommand;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
			.asSubsystemCommand(new InitExecuteCommand(() -> arm.setTargetPosition(position), () -> {}), "Set target position to: " + position);
	}

	public Command moveToPosition(
		Supplier<Rotation2d> position,
		Rotation2d maxVelocityRotation2dPerSecond,
		Rotation2d maxAccelerationRotation2dPerSecondSquared
	) {
		return arm.asSubsystemCommand(
			new InitExecuteCommand(
				() -> {},
				() -> arm.setTargetPosition(position.get(), maxVelocityRotation2dPerSecond, maxAccelerationRotation2dPerSecondSquared)
			),
			"Set target position to: " + position
		);
	}

	public Command moveToPosition(
		Rotation2d position,
		Rotation2d maxVelocityRotation2dPerSecond,
		Rotation2d maxAccelerationRotation2dPerSecondSquared
	) {
		return arm.asSubsystemCommand(
			new InitExecuteCommand(
				() -> arm.setTargetPosition(position, maxVelocityRotation2dPerSecond, maxAccelerationRotation2dPerSecondSquared),
				() -> {}
			),
			"Set target position to: " + position
		);
	}

	public Command stayInPlace() {
		return arm.asSubsystemCommand(new InitExecuteCommand(arm::stayInPlace, () -> {}), "Stay in place");
	}

	public Command loggedDashboardSetVoltage() {
		return arm.asSubsystemCommand(new LoggedDashboardCommand("Set Arm Voltage", arm::setVoltage), "Set voltage via dashboard");
	}

}
