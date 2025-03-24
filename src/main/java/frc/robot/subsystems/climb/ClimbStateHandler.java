package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.joysticks.SmartJoystick;
import frc.robot.subsystems.climb.lifter.LifterState;
import frc.robot.subsystems.climb.lifter.LifterStateHandler;
import frc.robot.subsystems.climb.solenoid.SolenoidState;
import frc.robot.subsystems.climb.solenoid.SolenoidStateHandler;

public class ClimbStateHandler {

	private final SolenoidStateHandler solenoidStateHandler;
	private final LifterStateHandler lifterStateHandler;
	private ClimbState currentState;

	public ClimbStateHandler(SolenoidStateHandler solenoidStateHandler, LifterStateHandler lifterStateHandler) {
		this.solenoidStateHandler = solenoidStateHandler;
		this.lifterStateHandler = lifterStateHandler;
	}

	public ClimbState getCurrentState() {
		return currentState;
	}

	public Command setState(ClimbState state) {
		return new ParallelCommandGroup(new InstantCommand(() -> currentState = state), switch (state) {
			case STOP -> stop();
			case DEPLOY -> deploy();
			case CLIMB_WITHOUT_LIMIT_SWITCH -> climbWithoutLimitSwitch();
			case CLIMB_WITH_LIMIT_SWITCH -> climbWithLimitSwitch();
			case MANUAL_CLIMB -> manualClimb();
			case CLOSE -> close();
			case EXIT_CLIMB -> exitClimb();
		});
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

	private Command climbWithoutLimitSwitch() {
		return new SequentialCommandGroup(
			new ParallelCommandGroup(lifterStateHandler.setState(LifterState.CLIMB), solenoidStateHandler.setState(SolenoidState.LOCKED))
				.until(() -> lifterStateHandler.isLower(LifterState.CLIMB.getTargetPosition())),
			new ParallelCommandGroup(lifterStateHandler.setState(LifterState.HOLD), solenoidStateHandler.setState(SolenoidState.LOCKED))
		);
	}

	private Command climbWithLimitSwitch() {
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				new InstantCommand(() -> lifterStateHandler.getLifter().setBrake(true)),
				lifterStateHandler.setState(LifterState.CLIMB).until(solenoidStateHandler::isAtLimitSwitch).until(() -> lifterStateHandler.isLower(Rotation2d.fromDegrees(-5))),
				new InstantCommand(
						() -> lifterStateHandler.getLifter().position = lifterStateHandler.getLifter().getPosition())
						.alongWith(lifterStateHandler.setState(LifterState.CLIMB))
						.until(() -> lifterStateHandler.isLower(Rotation2d.fromDegrees(lifterStateHandler.getLifter().position.getDegrees() - 1.5))).until(() -> lifterStateHandler.isLower(Rotation2d.fromDegrees(-5)))
					,
				lifterStateHandler.setState(LifterState.HOLD)
			),
			solenoidStateHandler.setState(SolenoidState.LOCKED)
		);
	}

	private Command manualClimb() {
		return new ParallelCommandGroup(
			lifterStateHandler.setState(LifterState.MANUAL_CLIMB),
			solenoidStateHandler.setState(SolenoidState.LOCKED)
		);
	}

	private Command exitClimb() {
		return new SequentialCommandGroup(
			new ParallelCommandGroup(
				lifterStateHandler.setState(LifterState.BACKWARD).withTimeout(ClimbConstants.SOLENOID_RELEASE_TIME_SECONDS),
				solenoidStateHandler.setState(SolenoidState.INITIAL_FREE)
					.withTimeout(ClimbConstants.SOLENOID_RETRACTING_UNTIL_HOLDING_TIME_SECONDS)
			),
			new ParallelCommandGroup(lifterStateHandler.setState(LifterState.FORWARD), solenoidStateHandler.setState(SolenoidState.HOLD_FREE))

		);
	}

	private Command close() {
		return new SequentialCommandGroup(
			new ParallelCommandGroup(lifterStateHandler.setState(LifterState.CLOSE), solenoidStateHandler.setState(SolenoidState.LOCKED))
				.until(() -> lifterStateHandler.isLower(LifterState.CLOSE.getTargetPosition())),
			new ParallelCommandGroup(lifterStateHandler.setState(LifterState.HOLD), solenoidStateHandler.setState(SolenoidState.LOCKED))
		);
	}

	public void applyCalibrationBindings(SmartJoystick joystick) {
		joystick.X.onTrue(setState(ClimbState.CLIMB_WITHOUT_LIMIT_SWITCH));
		joystick.Y.whileTrue(setState(ClimbState.MANUAL_CLIMB));
		joystick.B.onTrue(setState(ClimbState.DEPLOY));
		joystick.A.onTrue(setState(ClimbState.STOP));
	}

}
