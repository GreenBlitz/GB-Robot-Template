package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj2.command.Command;

public class EndEffectorStateHandler {

	private final EndEffector endEffector;

	public EndEffectorStateHandler(EndEffector endEffector) {
		this.endEffector = endEffector;
	}

	public Command setState(EndEffectorState state) {
		return switch (state) {
			case IDLE -> endEffector.getCommandsBuilder().stop();
			case INTAKE -> endEffector.getCommandsBuilder().setPower(state.getPower()).until(endEffector::isCoralInBack);
			case OUTTAKE -> endEffector.getCommandsBuilder().setPower(state.getPower()).until(() -> !endEffector.isCoralInFront());
		};
	}

}
