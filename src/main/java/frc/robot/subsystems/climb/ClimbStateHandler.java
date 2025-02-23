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
			case DEPLOY -> deploy();
			case CLIMB -> climb();
			case CLOSE -> close();
		};
	}

	private Command stop() {
		return new ParallelCommandGroup(lifterStateHandler.setState(LifterState.HOLD), solenoidStateHandler.setState(SolenoidState.LOCKED));
	}

	private Command deploy() {
		return new SequentialCommandGroup(
			lifterStateHandler.setState(LifterState.BACKWARD).withTimeout(ClimbConstants.SOLENOID_RELEASE_TIME_SECONDS),
			solenoidStateHandler.setState(SolenoidState.INITIAL_FREE).withTimeout(ClimbConstants.SOLENOID_RETRACTING_UNTIL_HOLDING_TIME_SECONDS),
			new ParallelCommandGroup(lifterStateHandler.setState(LifterState.DEPLOY), solenoidStateHandler.setState(SolenoidState.HOLD_FREE))
				.until(() -> lifterStateHandler.isHigher(LifterState.DEPLOY.getTargetPosition())),
			new ParallelCommandGroup(solenoidStateHandler.setState(SolenoidState.LOCKED), lifterStateHandler.setState(LifterState.HOLD))
		);
	}

	private Command climb() {
		return new SequentialCommandGroup(
			new ParallelCommandGroup(lifterStateHandler.setState(LifterState.CLIMB), solenoidStateHandler.setState(SolenoidState.LOCKED))
				.until(() -> lifterStateHandler.isLower(LifterState.CLIMB.getTargetPosition())),
			new ParallelCommandGroup(lifterStateHandler.setState(LifterState.HOLD), solenoidStateHandler.setState(SolenoidState.LOCKED))
		);
	}

	private Command close() {
		return new SequentialCommandGroup(
			new ParallelCommandGroup(lifterStateHandler.setState(LifterState.CLOSE), solenoidStateHandler.setState(SolenoidState.LOCKED))
				.until(() -> lifterStateHandler.isLower(LifterState.CLOSE.getTargetPosition())),
			new ParallelCommandGroup(lifterStateHandler.setState(LifterState.HOLD), solenoidStateHandler.setState(SolenoidState.LOCKED))
		);
	}

}
