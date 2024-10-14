package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;

public class StatesMotionPlanner {

	private final Superstructure superstructure;

	public StatesMotionPlanner(Superstructure superstructure) {
		this.superstructure = superstructure;
	}

	public Command setState(RobotState state) {
		return switch (state) {
			case INTAKE, ARM_INTAKE, SPEAKER, TRANSFER_SHOOTER_TO_ARM, TRANSFER_ARM_TO_SHOOTER ->
				superstructure.setState(state)
					.until(superstructure::isEnableChangeStateAutomatically)
					.andThen(superstructure.setState(RobotState.IDLE));
			case INTAKE_OUTTAKE, AMP, ARM_OUTTAKE, PRE_AMP, PRE_SPEAKER, IDLE -> superstructure.setState(state);
		};
	}

}
