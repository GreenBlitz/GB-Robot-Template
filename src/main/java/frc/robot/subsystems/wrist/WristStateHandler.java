package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.superstructure.Tolerances;

public class WristStateHandler {
	
	private final Wrist wrist;
	
	public WristStateHandler(Wrist wrist) {
		this.wrist = wrist;
	}
	
	public Command setState(WristState state) {
		return wrist.getCommandsBuilder().moveToPosition(state.getPosition()).until(() -> wrist.isAtPosition(state.getPosition(), Tolerances.WRIST_POSITION_TOLERANCE));
	}
}
