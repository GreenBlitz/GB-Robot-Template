package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.superstructure.Tolerances;


public class RollerStateHandler {

	private final Roller roller;

	public RollerStateHandler(Roller roller) {
		this.roller = roller;
	}

	public Command setState(RollerState rollerState) {
		if (rollerState == RollerState.AFTER_INTAKE) {
			return roller.getCommandsBuilder().rollRotations(Tolerances.ROLLER_INTAKE_ROTATIONS, rollerState.getPower());
		}
		if (rollerState == RollerState.MANUAL) {
			return new InstantCommand();
		}
		return roller.getCommandsBuilder().moveByPower(rollerState.getPower());
	}

}
