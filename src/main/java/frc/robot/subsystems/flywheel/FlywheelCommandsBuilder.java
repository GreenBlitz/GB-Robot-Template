package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class FlywheelCommandsBuilder {

	private final Flywheel flywheel;

	public FlywheelCommandsBuilder(Flywheel flywheel) {
		this.flywheel = flywheel;
	}

	//@formatter:off
	public Command setPower(double power) {
		return new FunctionalCommand(
			() -> {},
			() -> flywheel.setPower(power),
			interrupted -> flywheel.stop(),
			() -> false,
			flywheel
		).withName("Set power to " + power);
	}

	public Command setVelocity(Rotation2d targetVelocity) {
		return new RunCommand(
			() -> flywheel.setTargetVelocity(targetVelocity),
			flywheel
		).withName("Set target velocity to " + targetVelocity);
	}

	public Command stop() {
		return new RunCommand(flywheel::stop, flywheel).withName("Stop");
	}
	//@formatter:on

}
