package frc.robot.subsystems.wrist;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import java.util.function.DoubleSupplier;

public class WristCommandsBuilder {
	
	private final Wrist wrist;
	
	public WristCommandsBuilder(Wrist wrist) {
		this.wrist = wrist;
	}
	
	public Command setPower(DoubleSupplier power) {
		return new FunctionalCommand(
				() -> {},
				() -> wrist.setPower(power.getAsDouble()),
				(interrupted) -> wrist.stop(),
				() -> false,
				wrist
		).withName("set power to: " + power);
	}
	
	public Command setPosition(Rotation2d position) {
		return new FunctionalCommand(
				() -> {},
				() -> wrist.setTargetPosition(position),
				(interrupted) -> wrist.stop(),
				() -> false,
				wrist
		).withName("set position to: " + position.getDegrees());
	}
}
