package frc.robot.subsystems.roller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

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
