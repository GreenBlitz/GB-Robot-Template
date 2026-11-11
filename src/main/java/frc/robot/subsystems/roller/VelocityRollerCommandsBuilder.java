package frc.robot.subsystems.roller;

import org.wpilib.command2.Command;
import org.wpilib.command2.RunCommand;
import frc.utils.angle2d.Angle2d;

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
