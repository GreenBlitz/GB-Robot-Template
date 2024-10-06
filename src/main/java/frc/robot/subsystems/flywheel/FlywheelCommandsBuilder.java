package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

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
			interrupted -> flywheel.stop(),
			() -> false,
			flywheel
		).withName("Set powers - right: " + rightPower + ", left: " + leftPower);
	}

	public Command setPowers(Supplier<Double> rightPower, Supplier<Double> leftPower) {
		return new FunctionalCommand(
			() -> {},
			() -> flywheel.setPowers(rightPower.get(), leftPower.get()),
			interrupted -> flywheel.stop(),
			() -> false,
			flywheel
		).withName("Set power by supplier");
	}

	public Command setVelocities(Rotation2d rightVelocity, Rotation2d leftVelocity) {
		return new FunctionalCommand(
			() -> flywheel.setTargetVelocities(rightVelocity, leftVelocity),
			() -> {},
			interrupted -> flywheel.stop(),
			() -> false,
			flywheel
		).withName("Set velocities - right: " + rightVelocity + ", left: " + leftVelocity);
	}

	public Command stop() {
		return new RunCommand(flywheel::stop, flywheel).withName("Stop");
	}
	//@formatter:on

}
