package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.utils.utilcommands.InitExecuteCommand;

public class ExampleArmCommandsBuilder {

	private final ExampleArm arm;

	public ExampleArmCommandsBuilder(ExampleArm arm) {
		this.arm = arm;
	}

	public Command moveToPosition(Rotation2d position) {
		return arm.asSubsystemCommand(new InitExecuteCommand(() -> arm.setTargetPosition(position), () -> {}), "Move to position: " + position);
	}

	public Command stayInPlace() {
		return arm.asSubsystemCommand(new RunCommand(arm::stayInPlace), "Stay in place");
	}

	public Command setVoltage(double voltage) {
		return arm.asSubsystemCommand(new RunCommand(() -> arm.setVoltage(voltage)), "Set voltage to: " + voltage);
	}

	public Command setPower(double power){
		return arm.asSubsystemCommand(new RunCommand(() -> arm.setPower(power)), "Set power to: " + power);
	}


}
