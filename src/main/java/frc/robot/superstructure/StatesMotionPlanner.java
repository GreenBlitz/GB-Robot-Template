package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;

public class StatesMotionPlanner {

	private final Superstructure superstructure;

	public StatesMotionPlanner(Superstructure superstructure) {
		this.superstructure = superstructure;
	}

	public Command intakeTransfertoarmAmp() {
		return superstructure.setState(RobotState.INTAKE)
			.andThen(superstructure.setState(RobotState.TRANSFER_SHOOTER_TO_ARM))
			.andThen(superstructure.setState(RobotState.AMP));
	}

}
