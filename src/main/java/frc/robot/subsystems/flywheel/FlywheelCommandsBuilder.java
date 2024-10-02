package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.utils.utilcommands.InitExecuteCommand;

public class FlywheelCommandsBuilder {

	private Flywheel flywheel;

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
		).withName("Set Flywheel power to " + power);
	}

	public Command setVelocity(Rotation2d targetVelocity) {
		return new InitExecuteCommand(
				() -> flywheel.setTargetVelocity(targetVelocity),
				() -> {},
				flywheel
		).withName("Set flywheel velocity set point to " + targetVelocity);
	}

	//@formatter:on

	public Command stop() {
		return new RunCommand(flywheel::stop, flywheel).withName("Stop flywheel");
	}

}
