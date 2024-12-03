package frc.robot.subsystems.motorsubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.Supplier;

public class MotorCommandsBuilder {

	private MotorSubsystem motorSubsystem;

	public MotorCommandsBuilder(MotorSubsystem motorSubsystem) {
		motorSubsystem = motorSubsystem;
	}

	public Command moveToAngle(Rotation2d angle) {
		return new RunCommand(() -> motorSubsystem.setTargetPosition(angle), motorSubsystem);
	}

	public Command moveToAngle(Supplier<Rotation2d> angleSupplier) {
		return new RunCommand(() -> motorSubsystem.setTargetPosition(angleSupplier.get()), motorSubsystem);
	}

	public Command stayInPlace() {
		return new RunCommand(() -> motorSubsystem.stayInPlace()).withName("Stay in place");
	}

}
