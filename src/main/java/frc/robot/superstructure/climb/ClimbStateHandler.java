package frc.robot.superstructure.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.lifter.LifterState;
import frc.robot.subsystems.lifter.LifterStateHandler;
import frc.robot.subsystems.solenoid.SolenoidState;
import frc.robot.subsystems.solenoid.SolenoidStateHandler;

public class ClimbStateHandler {

	private final LifterStateHandler lifterStateHandler;
	private final SolenoidStateHandler solenoidStateHandler;

	public ClimbStateHandler(LifterStateHandler lifterStateHandler, SolenoidStateHandler solenoidStateHandler) {
		this.lifterStateHandler = lifterStateHandler;
		this.solenoidStateHandler = solenoidStateHandler;
	}

	public Command setState(ClimbState state) {
		return switch (state) {
			case STOP -> lifterStateHandler.setState(LifterState.HOLD).alongWith(solenoidStateHandler.setState(SolenoidState.OFF));
			case EXTEND ->
				(lifterStateHandler.setState(LifterState.BACKWARD).raceWith(new WaitCommand(ClimbConstants.SOLENOID_RELEASE_TIME_SECONDS)))
					.andThen(
						(solenoidStateHandler.setState(SolenoidState.RETRACT))
							.raceWith(new WaitCommand(ClimbConstants.SOLENOID_RETRACTING_UNTIL_HOLDING_TIME))
					)
					.andThen(
						lifterStateHandler.setState(LifterState.EXTENDED)
							.alongWith(
								solenoidStateHandler.setState(SolenoidState.HOLD)
									.raceWith(new WaitCommand(ClimbConstants.SOLENOID_TIMEOUT_SECONDS))
									.andThen(solenoidStateHandler.setState(SolenoidState.OFF))
							)
					);
			case RETRACT -> lifterStateHandler.setState(LifterState.RETRACTED).alongWith(solenoidStateHandler.setState(SolenoidState.OFF));
		};
	}

}
