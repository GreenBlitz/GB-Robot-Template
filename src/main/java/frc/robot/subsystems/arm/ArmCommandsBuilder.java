package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.utils.utilcommands.LoggedDashboardCommand;

public class ArmCommandsBuilder {

	private final Arm arm;

	public ArmCommandsBuilder(Arm arm) {
		this.arm = arm;
	}

	public Command moveToTargetPosition(Rotation2d position) {
		return arm.asSubsystemCommand(new RunCommand(() -> arm.setTargetPosition(position)), "Set target position to: " + position);
	}

	public Command stayInPlace() {
		return arm.asSubsystemCommand(new RunCommand(arm::stayInPlace), "Stay in place");
	}

	public Command setVoltage(double voltage) {
		return arm.asSubsystemCommand(new RunCommand(() -> arm.setVoltage(voltage)), "Set voltage to: " + voltage);
	}

	public Command setPower(double power) {
		return arm.asSubsystemCommand(new RunCommand(() -> arm.setPower(power)), "Set power to: " + power);
	}

	public Command loggedDashboardSetVoltage() {
		return arm.asSubsystemCommand(
			new LoggedDashboardCommand("Set Arm Voltage", (voltage) -> arm.setVoltage(voltage)),
			"Set voltage via dashboard"
		);
	}

}
