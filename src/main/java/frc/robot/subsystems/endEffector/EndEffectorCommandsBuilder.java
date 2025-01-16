package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.DoubleSupplier;

public class EndEffectorCommandsBuilder {

	private final EndEffector endEffector;

	public EndEffectorCommandsBuilder(EndEffector endEffector) {
		this.endEffector = endEffector;
	}

	public Command setPower(double power) {
		return new RunCommand(() -> endEffector.setPower(power), endEffector);
	}

	public Command setPower(DoubleSupplier powerSupplier) {
		return new RunCommand(() -> endEffector.setPower(powerSupplier.getAsDouble()), endEffector);
	}

	public Command stop() {
		return new InstantCommand(endEffector::stop, endEffector);
	}

}
