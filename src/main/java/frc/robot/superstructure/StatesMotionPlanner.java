package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class StatesMotionPlanner {

	private final Superstructure superstructure;

	public StatesMotionPlanner(Superstructure superstructure) {
		this.superstructure = superstructure;
	}

	public Command setState(RobotState state) {
		return switch (state) {
			case INTAKE, ARM_INTAKE, SPEAKER, TRANSFER_SHOOTER_TO_ARM, TRANSFER_ARM_TO_SHOOTER, PASSING ->
				new SequentialCommandGroup(
					superstructure.enableChangeStateAutomatically(false),
					superstructure.setState(state).until(superstructure::isEnableChangeStateAutomatically),
					superstructure.setState(RobotState.IDLE)
				);
			case INTAKE_OUTTAKE, AMP, ARM_OUTTAKE, PRE_AMP, PRE_SPEAKER, IDLE -> superstructure.setState(state);
		};
	}

}
