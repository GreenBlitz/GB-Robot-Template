package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ShooterStateHandler {

	public ShooterStateHandler() {}

	public Command setState(ShooterState robotState) {
		return new InstantCommand();
	}

	public enum ShooterState {

		STAY_IN_PLACE,
		IDLE,
		SHOOT,
		PRE_SHOOT;

	}

}
