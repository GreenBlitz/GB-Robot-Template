package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.Supplier;

public class FlywheelCommandsBuilder {

	private final Flywheel flywheel;

	public FlywheelCommandsBuilder(Flywheel flywheel) {
		this.flywheel = flywheel;
	}

	//@formatter:off
	public Command setPowers(double rightPower, double leftPower) {
		return new FunctionalCommand(
			() -> {},
			() -> flywheel.setPowers(rightPower, leftPower),
			(interrupted) -> flywheel.stop(),
			() -> false,
			flywheel
		).withName("Set powers - right: " + rightPower + ", left: " + leftPower);
	}

	public Command setPowers(Supplier<Double> rightPower, Supplier<Double> leftPower) {
		return new FunctionalCommand(
			() -> {},
			() -> flywheel.setPowers(rightPower.get(), leftPower.get()),
			(interrupted) -> {},
			() -> false,
			flywheel
		).withName("Set power by supplier");
	}

	public Command setVelocities(Rotation2d rightVelocity, Rotation2d leftVelocity, Rotation2d tolerance) {
		return new FunctionalCommand(
			() -> flywheel.setTargetVelocities(rightVelocity, leftVelocity),
			() -> {},
			(interrupted) -> {},
			() -> flywheel.isAtVelocities(rightVelocity, leftVelocity, tolerance),
			flywheel
		).withName("Set velocities - right: " + rightVelocity + ", left: " + leftVelocity);
	}
	//@formatter:on

	public Command stop() {
		return new InstantCommand(flywheel::stop, flywheel).withName("Stop");
	}

}
