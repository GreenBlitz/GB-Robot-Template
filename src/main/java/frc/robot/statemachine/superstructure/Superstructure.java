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

		this.currentState = RobotState.STAY_IN_PLACE;
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
			case DRIVE -> idle();
			case INTAKE -> intake();
			case PRE_SHOOT -> preShoot();
			case SHOOT -> shoot();
			case SHOOT_AND_INTAKE -> shootAndIntake();
		};
	}

	private Command stayInPlace() {
		return new ParallelCommandGroup(
			shooterStateHandler.setState(ShooterState.STAY_IN_PLACE),
			funnelStateHandler.setState(FunnelState.STAY_IN_PLACE),
			intakeStateHandler.setState(IntakeState.STAY_IN_PLACE)
		);
	}

	private Command idle() {
		return new ParallelCommandGroup(
			shooterStateHandler.setState(ShooterState.IDLE),
			funnelStateHandler.setState(FunnelState.DRIVE),
			intakeStateHandler.setState(IntakeState.CLOSED)
		);
	}

	private Command intake() {
		return new ParallelCommandGroup(
			shooterStateHandler.setState(ShooterState.IDLE),
			funnelStateHandler.setState(FunnelState.INTAKE),
			intakeStateHandler.setState(IntakeState.INTAKE)
		);
	}

	private Command preShoot() {
		return new ParallelCommandGroup(
			shooterStateHandler.setState(ShooterState.SHOOT),
			funnelStateHandler.setState(FunnelState.DRIVE),
			intakeStateHandler.setState(IntakeState.CLOSED)
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

	public void log() {
		Logger.recordOutput(logPath + "/IsSubsystemRunningIndependently", isSubsystemRunningIndependently());
	}

}
