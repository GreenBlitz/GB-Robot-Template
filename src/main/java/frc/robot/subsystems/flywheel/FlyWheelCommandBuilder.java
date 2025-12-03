package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.GBCommandsBuilder;

import java.util.function.Supplier;

public class FlyWheelCommandBuilder extends GBCommandsBuilder {

	private final FlyWheel flyWheel;

	public FlyWheelCommandBuilder(FlyWheel flyWheel) {
		super();
		this.flyWheel = flyWheel;
	}

	public Command setTargetVelocity(Rotation2d velocity) {
		return flyWheel.asSubsystemCommand(
			new RunCommand(() -> flyWheel.setTargetVelocity(velocity)),
			"set velocity to " + velocity.getRotations() + " rotations"
		);
	}

	public Command setVelocityAsSupplier(Supplier<Rotation2d> velocity) {
		return flyWheel.asSubsystemCommand(
			new RunCommand(() -> flyWheel.setTargetVelocity(velocity.get())),
			"set velocity as supplier"

		);
	}

	public Command stop() {
		return flyWheel.asSubsystemCommand(new RunCommand(flyWheel::stop), "stop");
	}

}
