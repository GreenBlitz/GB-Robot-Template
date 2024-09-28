package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class RollerStateHandler {

	private final Roller roller;

	public RollerStateHandler(Roller roller) {
		this.roller = roller;
	}

	public Command setState(RollerState rollerState) {
		if (rollerState == RollerState.INTAKE) {
			return new SequentialCommandGroup(
				roller.getCommandsBuilder().moveByPower(rollerState.getPower()),//.until(roller::isObjectIn),
				roller.getCommandsBuilder().rollRotations(RollerConstants.INTAKE_ROTATIONS, rollerState.getPower())
			).withTimeout(0.7);
		}
		if (rollerState == RollerState.MANUAL) {
			return new InstantCommand();
		}
		return roller.getCommandsBuilder().moveByPower(rollerState.getPower());
	}

}
