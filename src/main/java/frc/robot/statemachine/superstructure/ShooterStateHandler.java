package frc.robot.statemachine.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterStateHandler {

	public ShooterStateHandler() {}

	public Command setState(ShooterState robotState) {
		return new InstantCommand();
	}

}
