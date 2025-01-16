package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj2.command.Command;

public class EndEffectorStateHandler {

	private final EndEffector endEffector;

	public EndEffectorStateHandler(EndEffector endEffector) {
		this.endEffector = endEffector;
	}

	public Command setState(EndEffectorState state) {
		if (state == EndEffectorState.IDLE) {
			return endEffector.getCommandsBuilder().stop();
		} else {
			return endEffector.getCommandsBuilder().setPower(state.getPower());
		}
	}

}
