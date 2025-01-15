package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.util.function.DoubleSupplier;

public class EndEffectorCommandsBuilder {

	private final EndEffector endEffector;

	public EndEffectorCommandsBuilder(EndEffector endEffector) {
		this.endEffector = endEffector;
	}

	public Command setPower(double power) {
		return new InstantCommand(() -> endEffector.setPower(power), endEffector);
	}

	public Command setPower(DoubleSupplier powerSupplier) {
		return new RunCommand(() -> endEffector.setPower(powerSupplier.getAsDouble()), endEffector);
	}

	public Command stop() {
		return new InstantCommand(endEffector::stop, endEffector);
	}

	public Command intake() {
		Command command = new FunctionalCommand(
			() -> endEffector.setPower(EndEffectorConstants.INTAKE_POWER),
			() -> {},
			interrupted -> endEffector.stop(),
			endEffector::isCoralInBackBeamBreaker,
			endEffector
		);

		return endEffector.asSubsystemCommand(command, "end effector intake");
	}

	public Command outtake() {
		Command command = new FunctionalCommand(
			() -> endEffector.setPower(EndEffectorConstants.OUTTAKE_POWER),
			() -> {},
			interrupted -> endEffector.stop(),
			endEffector::isCoralInFrontBeamBreaker,
			endEffector
		);

		return endEffector.asSubsystemCommand(command, "end effector outtake");
	}


}
