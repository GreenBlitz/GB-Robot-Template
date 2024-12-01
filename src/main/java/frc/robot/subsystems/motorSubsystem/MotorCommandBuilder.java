package frc.robot.subsystems.motorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.utils.utilcommands.ExecuteEndCommand;

import java.util.function.DoubleSupplier;

public class MotorCommandBuilder {

	private MotorSubsystem motor;

	public MotorCommandBuilder(MotorSubsystem motor) {
		this.motor = motor;
	}

	public Command setPower(DoubleSupplier powerSupplier) {
		return new ExecuteEndCommand(() -> motor.setPower(powerSupplier.getAsDouble()), () -> {}, motor);
	}

	public Command setPower(double power) {
		return new RunCommand(() -> motor.setPower(power));
	}

	public Command stop() {
		return new RunCommand(() -> motor.stop());
	}

}
