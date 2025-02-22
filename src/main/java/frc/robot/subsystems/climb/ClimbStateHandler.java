package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.climb.lifter.LifterState;
import frc.robot.subsystems.climb.lifter.LifterStateHandler;
import frc.robot.subsystems.climb.solenoid.SolenoidState;
import frc.robot.subsystems.climb.solenoid.SolenoidStateHandler;

public class ClimbStateHandler {

	private final SolenoidStateHandler solenoidStateHandler;
	private final LifterStateHandler lifterStateHandler;

	public ClimbStateHandler(SolenoidStateHandler solenoidStateHandler, LifterStateHandler lifterStateHandler) {
		this.solenoidStateHandler = solenoidStateHandler;
		this.lifterStateHandler = lifterStateHandler;
	}

	public Command setState(ClimbState state) {
		return switch (state) {
			case STOP -> stop();
			case EXTEND -> extend();
			case RETRACT -> retract();
		};
	}

	private Command stop() {
		return new ParallelCommandGroup(lifterStateHandler.setState(LifterState.HOLD), solenoidStateHandler.setState(SolenoidState.LOCKED));
	}

	private Command extend() {
		return new SequentialCommandGroup(
			lifterStateHandler.setState(LifterState.BACKWARD).withTimeout(ClimbConstants.SOLENOID_RELEASE_TIME_SECONDS),
			solenoidStateHandler.setState(SolenoidState.INITIAL_FREE).withTimeout(ClimbConstants.SOLENOID_RETRACTING_UNTIL_HOLDING_TIME_SECONDS),
			new ParallelDeadlineGroup(lifterStateHandler.setState(LifterState.DEPLOY), solenoidStateHandler.setState(SolenoidState.HOLD_FREE)),
			solenoidStateHandler.setState(SolenoidState.LOCKED)
		);
	}

	private Command retract() {
		return new ParallelCommandGroup(lifterStateHandler.setState(LifterState.CLIMB), solenoidStateHandler.setState(SolenoidState.LOCKED));
	}

}
