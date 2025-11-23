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

	private final IntakeStateHandler intakeStateHandler;
	private final FunnelStateHandler funnelStateHandler;
	private final ShooterStateHandler shooterStateHandler;

	private RobotState currentState;

	public Superstructure(String logPath, Robot robot) {
		this.robot = robot;

		this.subsystems = Set.of();

		this.targetChecks = new TargetChecks(this);

		this.currentState = null;
		this.isSubsystemRunningIndependently = false;
		this.logPath = logPath;

		this.funnelStateHandler = new FunnelStateHandler();
		this.intakeStateHandler = new IntakeStateHandler();
		this.shooterStateHandler = new ShooterStateHandler();
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
			case STAY_IN_PLACE -> stayInPlace();
			case INTAKE -> intake();
			case SHOOT -> shoot();
			case DRIVE -> idle();
			case SHOOT_AND_INTAKE -> shootAndIntake();
			case PRE_SHOOT -> preShoot();
		};
	}

	private Command stayInPlace() {
		return new ParallelCommandGroup(
			shooterStateHandler.setState(ShooterState.STAY_IN_PLACE),
			funnelStateHandler.setState(FunnelState.STAY_IN_PLACE),
			intakeStateHandler.setState(IntakeState.INTAKE)
		);
	}

	private Command intake() {
		return new ParallelCommandGroup(
			shooterStateHandler.setState(ShooterState.IDLE),
			funnelStateHandler.setState(FunnelState.INTAKE),
			intakeStateHandler.setState(IntakeState.INTAKE)
		);
	}

	private Command shoot() {
		return new ParallelCommandGroup(
			shooterStateHandler.setState(ShooterState.SHOOT),
			funnelStateHandler.setState(FunnelState.SHOOT),
			intakeStateHandler.setState(IntakeState.CLOSED)
		);
	}

	private Command shootAndIntake() {
		return new ParallelCommandGroup(
			shooterStateHandler.setState(ShooterState.SHOOT),
			funnelStateHandler.setState(FunnelState.SHOOT),
			intakeStateHandler.setState(IntakeState.INTAKE)
		);
	}

	private Command idle() {
		return new ParallelCommandGroup(
			shooterStateHandler.setState(ShooterState.IDLE),
			funnelStateHandler.setState(FunnelState.DRIVE),
			intakeStateHandler.setState(IntakeState.CLOSED)
		);
	}

	private Command preShoot() {
		return new ParallelCommandGroup(
			shooterStateHandler.setState(ShooterState.SHOOT),
			funnelStateHandler.setState(FunnelState.STAY_IN_PLACE),
			intakeStateHandler.setState(IntakeState.CLOSED)
		);
	}

	public void log() {
		Logger.recordOutput(logPath + "/IsSubsystemRunningIndependently", isSubsystemRunningIndependently());
	}

}
