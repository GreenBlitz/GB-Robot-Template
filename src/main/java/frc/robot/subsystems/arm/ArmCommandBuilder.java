package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.utils.utilcommands.InitExecuteCommand;

public class ArmCommandBuilder {

	private final Arm arm;

	public ArmCommandBuilder(Arm arm) {
		this.arm = arm;
	}

	public Command moveToPosition(Rotation2d position) {
		return arm.asSubsystemCommand(new InitExecuteCommand(() -> arm.setTargetPosition(position), () -> {}), "Move to position: " + position);
	}

	public Command stayInPlace() {
		return new RunCommand(arm::stayInPlace, arm).withName("Stay in place");
	}

	public Command setVoltage(double voltage) {
		return new RunCommand(() -> arm.setVoltage(voltage), arm).withName("Set voltage to: " + voltage);
	}


}
