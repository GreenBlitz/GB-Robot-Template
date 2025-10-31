package frc.robot.statemachine.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.statemachine.RobotState;
import frc.robot.subsystems.GBSubsystem;

import org.littletonrobotics.junction.Logger;

import java.util.Set;

public class Superstructure extends GBSubsystem {

	private final Robot robot;
	private final Set<Subsystem> subsystems;
	private final TargetChecks targetChecks;
	private boolean isRunningIndependently;

	private RobotState currentState;

	public Superstructure(String logPath, Robot robot) {
		super(logPath);
		this.robot = robot;

		this.subsystems = Set.of(this);

		this.targetChecks = new TargetChecks(this);

		this.currentState = RobotState.STAY_IN_PLACE;
		this.isRunningIndependently = false;
	}

	public boolean isRunningIndependently() {
		return isRunningIndependently;
	}

	public void setIsRunningIndependently(boolean isRunningIndependently) {
		this.isRunningIndependently = isRunningIndependently;
	}

	public RobotState getCurrentState() {
		return currentState;
	}

	public TargetChecks getTargetChecks() {
		return targetChecks;
	}

	@Override
	protected void subsystemPeriodic() {
		log();
	}

	public Command setState(RobotState robotState) {
		return switch (robotState) {
			case STAY_IN_PLACE -> stayInPlace();
		};
	}

	private void log() {
		Logger.recordOutput(getLogPath() + "/IsRunningIndependently", isRunningIndependently());
	}

	public Command stayInPlace() {
		return asSubsystemCommand(Commands.none(), RobotState.STAY_IN_PLACE);
	}

	private Command asSubsystemCommand(Command command, RobotState state) {
		return new ParallelCommandGroup(asSubsystemCommand(command, state.name()), new InstantCommand(() -> currentState = state));
	}

}
