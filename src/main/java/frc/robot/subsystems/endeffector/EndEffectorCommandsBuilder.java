package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.DoubleSupplier;

public class EndEffectorCommandsBuilder {

	private final EndEffector endEffector;

	public EndEffectorCommandsBuilder(EndEffector endEffector) {
		this.endEffector = endEffector;
	}

	public Command stop() {
		return setPower(0);
	}

	public Command setPower(double power) {
		return endEffector.asSubsystemCommand(new RunCommand(() -> endEffector.setPower(power)), "Set power to " + power);
	}

	public Command setPower(DoubleSupplier powerSupplier) {
		return endEffector.asSubsystemCommand(new RunCommand(() -> endEffector.setPower(powerSupplier.getAsDouble())), "Set power by supplier");
	}

}
