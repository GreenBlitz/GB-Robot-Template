package frc.robot.statemachine.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.statemachine.RobotState;

import org.littletonrobotics.junction.Logger;

import java.util.Set;

public class Superstructure {

	private final Robot robot;
	private final Set<Subsystem> subsystems;
	private final TargetChecks targetChecks;
	private boolean isSubsystemRunningIndependently;
	private String logPath;

	private RobotState currentState;

	public Superstructure(String logPath, Robot robot) {
		this.robot = robot;

		this.subsystems = Set.of();

		this.targetChecks = new TargetChecks(this);

		this.currentState = null;
		this.isSubsystemRunningIndependently = false;
		this.logPath = logPath;
	}

	public boolean isSubsystemRunningIndependently() {
		return isSubsystemRunningIndependently;
	}

	public void setIsSubsystemRunningIndependently(boolean isSubsystemRunningIndependently) {
		this.isSubsystemRunningIndependently = isSubsystemRunningIndependently;
	}

	public RobotState getCurrentState() {
		return currentState;
	}

	public TargetChecks getTargetChecks() {
		return targetChecks;
	}

	public Command setState(RobotState robotState) {
		return switch (robotState) {
			default -> new InstantCommand();
		};
	}

	public void log() {
		Logger.recordOutput(logPath + "/IsSubsystemRunningIndependently", isSubsystemRunningIndependently());
	}

}
