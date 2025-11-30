package frc.robot.statemachine.funnelstatehandler;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FunnelStateHandler {

	public FunnelStateHandler() {}

	public Command setState(FunnelState robotState) {
		return new InstantCommand();
	}

}
