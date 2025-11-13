package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.Supplier;

public class FlyWheelCommandBuilder {

	private final FlyWheel flyWheel;

	public FlyWheelCommandBuilder(FlyWheel flyWheel) {
		this.flyWheel = flyWheel;
		flyWheel.setDefaultCommand(stop());
	}

	public Command setVelocity(Rotation2d velocity) {
		return new RunCommand(() -> flyWheel.setVelocity(velocity), flyWheel);
	}

	public Command setVelocityAsSupplier(Supplier<Rotation2d> velocity) {
		return new RunCommand(() -> flyWheel.setVelocity((velocity.get())), flyWheel);
	}

	public Command stop() {
		return new RunCommand(flyWheel::stop, flyWheel);
	}

}

