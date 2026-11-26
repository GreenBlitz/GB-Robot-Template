package frc.robot.subsystems.roller;

import frc.utils.angle2d.Angle2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class VelocityRollerCommandsBuilder extends RollerCommandsBuilder {

	private final VelocityRoller roller;

	public VelocityRollerCommandsBuilder(VelocityRoller roller) {
		super(roller);
		this.roller = roller;
	}

	public Command setVelocity(Angle2d velocityAPS) {
		return roller.asSubsystemCommand(
			new RunCommand(() -> roller.setVelocity(velocityAPS)),
			"set velocity to " + velocityAPS.getRotations() + "rot/s"
		);
	}

}
