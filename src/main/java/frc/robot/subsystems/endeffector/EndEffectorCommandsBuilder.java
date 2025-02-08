package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.utils.utilcommands.ExecuteEndCommand;

import java.util.function.DoubleSupplier;

public class EndEffectorCommandsBuilder {

	private final EndEffector endEffector;

	public EndEffectorCommandsBuilder(EndEffector endEffector) {
		this.endEffector = endEffector;
	}

	public Command stop() {
		return endEffector.asSubsystemCommand(new RunCommand(endEffector::stop), "Stop");
	}

	public Command setPower(double power) {
		return endEffector.asSubsystemCommand(
			new ExecuteEndCommand(() -> endEffector.setPower(power), () -> endEffector.setPower(0)),
			"Set power to " + power
		);
	}

	public Command setPower(DoubleSupplier powerSupplier) {
		return endEffector.asSubsystemCommand(new RunCommand(() -> endEffector.setPower(powerSupplier.getAsDouble())), "Set power by supplier");
	}

}
