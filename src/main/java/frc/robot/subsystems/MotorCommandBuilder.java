package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import java.util.function.Supplier;

public class MotorCommandBuilder {

	MotorSubsystemPosition motorSubsystemPosition;

	public MotorCommandBuilder() {
		motorSubsystemPosition = new MotorSubsystemPosition(motorSubsystemPosition.motor, motorSubsystemPosition.getLogPath());
	}

	public Command moveToAngle(Rotation2d angle) {
		return new FunctionalCommand(
			() -> {},
			() -> motorSubsystemPosition.setTargetPosition(angle),
			interrupted -> motorSubsystemPosition.stayInPlace(),
			() -> false,
			motorSubsystemPosition
		);
	}

	public Command moveToAngle(Supplier<Rotation2d> angleSupplier) {
		return new FunctionalCommand(
			() -> {},
			() -> motorSubsystemPosition.setTargetPosition(angleSupplier.get()),
			interrupted -> motorSubsystemPosition.stayInPlace(),
			() -> false,
			motorSubsystemPosition
		);
	}


}
