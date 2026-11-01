package frc.robot.subsystems.roller;

import org.wpilib.command2.Command;
import org.wpilib.command2.RunCommand;
import org.wpilib.math.geometry.Rotation2d;

public class VelocityRollerCommandsBuilder extends RollerCommandsBuilder {

	private final VelocityRoller roller;

	public VelocityRollerCommandsBuilder(VelocityRoller roller) {
		super(roller);
		this.roller = roller;
	}

	public Command setVelocity(Rotation2d velocityRPS) {
		return roller.asSubsystemCommand(
			new RunCommand(() -> roller.setVelocity(velocityRPS)),
			"set velocity to " + velocityRPS.getRotations() + "rot/s"
		);
	}

}
