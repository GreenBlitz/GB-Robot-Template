package frc.robot.statemachine.superstructure.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.lifter.LifterState;
import frc.robot.subsystems.lifter.LifterStateHandler;
import frc.robot.subsystems.solenoid.SolenoidState;
import frc.robot.subsystems.solenoid.SolenoidStateHandler;

public class ClimbStateHandler {

	private final SolenoidStateHandler solenoidStateHandler;
	private final LifterStateHandler lifterStateHandler;

	public ClimbStateHandler(SolenoidStateHandler solenoidStateHandler, LifterStateHandler lifterStateHandler) {
		this.solenoidStateHandler = solenoidStateHandler;
		this.lifterStateHandler = lifterStateHandler;
	}

	public Command setState(ClimbState state) {
		return switch (state) {
			case STOP -> lifterStateHandler.setState(LifterState.HOLD).alongWith(solenoidStateHandler.setState(SolenoidState.OFF));
			case EXTEND ->
				(lifterStateHandler.setState(LifterState.BACKWARD).withTimeout(ClimbConstants.SOLENOID_RELEASE_TIME_SECONDS))
					.andThen(
						(solenoidStateHandler.setState(SolenoidState.RETRACT)).withTimeout(ClimbConstants.SOLENOID_RETRACTING_UNTIL_HOLDING_TIME)
					)
					.andThen(lifterStateHandler.setState(LifterState.EXTENDED).raceWith(solenoidStateHandler.setState(SolenoidState.HOLD)))
					.andThen(solenoidStateHandler.setState(SolenoidState.OFF));
			case RETRACT -> lifterStateHandler.setState(LifterState.RETRACTED).alongWith(solenoidStateHandler.setState(SolenoidState.OFF));
		};
	}

}
