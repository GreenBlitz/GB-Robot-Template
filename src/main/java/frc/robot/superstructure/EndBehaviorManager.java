package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;

public class EndBehaviorManager {

	private final Superstructure superstructure;

	public EndBehaviorManager(Superstructure superstructure) {
		this.superstructure = superstructure;
	}

	public Command endState(RobotState state) {
		return switch (state) {
			case INTAKE, ARM_INTAKE, SPEAKER, TRANSFER_SHOOTER_TO_ARM, TRANSFER_ARM_TO_SHOOTER, PASSING ->
				superstructure.setState(RobotState.IDLE);
			case INTAKE_WITH_FLYWHEEL -> superstructure.setState(RobotState.PRE_SPEAKER);
			case AMP -> superstructure.setState(RobotState.ARM_UP);
			case INTAKE_OUTTAKE, ARM_OUTTAKE, PRE_AMP, PRE_SPEAKER, IDLE, ARM_UP -> superstructure.setState(state);
		};
	}

}
