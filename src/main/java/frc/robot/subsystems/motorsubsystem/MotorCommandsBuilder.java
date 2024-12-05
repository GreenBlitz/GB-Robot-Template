package frc.robot.subsystems.motorsubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.function.DoubleSupplier;

public class MotorCommandsBuilder {

	private final MotorSubsystem motorSubsystem;

	public MotorCommandsBuilder(MotorSubsystem motorSubsystem) {
		this.motorSubsystem = motorSubsystem;
	}

	public Command setPower(double power) {
		return new RunCommand(() -> motorSubsystem.setPower(power), motorSubsystem).withName("set power: " + power);
	}

	public Command setPower(DoubleSupplier powerSupplier) {
		return new RunCommand(() -> motorSubsystem.setPower(powerSupplier.getAsDouble()), motorSubsystem).withName("set power by supplier");
	}

	public Command stop() {
		return new RunCommand(() -> motorSubsystem.stop(), motorSubsystem).withName("stop");
	}

}
