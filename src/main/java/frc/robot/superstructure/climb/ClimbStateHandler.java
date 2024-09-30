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
			case STOP -> lifterStateHandler.setState(LifterState.STOP).alongWith(solenoidStateHandler.setState(SolenoidState.OFF));
			case EXTEND ->
				lifterStateHandler.setState(LifterState.BACKWARD)
					.raceWith(new WaitCommand(ClimbConstants.SOLENOID_RELEASE_TIME))
					.andThen(solenoidStateHandler.setState(SolenoidState.RETRACT))
					.andThen(new WaitCommand(ClimbConstants.SOLENOID_RETRACTING_UNTIL_HOLDING_TIME))
					.andThen(lifterStateHandler.setState(LifterState.EXTENDED));
			case RETRACT -> lifterStateHandler.setState(LifterState.BACKWARD).alongWith(solenoidStateHandler.setState(SolenoidState.OFF));
		};
	}


}
