package frc.robot.statemachine.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.statemachine.RobotState;
import frc.robot.statemachine.funnelstatehandler.FunnelState;
import frc.robot.statemachine.funnelstatehandler.FunnelStateHandler;
import frc.robot.statemachine.intakestatehandler.IntakeStateHandler;
import frc.robot.statemachine.shooterstatehandler.ShooterState;
import frc.robot.statemachine.shooterstatehandler.ShooterStateHandler;
import org.littletonrobotics.junction.Logger;

import java.util.Set;
import java.util.function.Supplier;

public class Superstructure {

	private final Robot robot;
	private final Set<Subsystem> subsystems;
	private final TargetChecks targetChecks;
	private boolean isSubsystemRunningIndependently;
	private String logPath;

	private RobotState currentState;

	private final IntakeStateHandler intakeStateHandler;
	private final FunnelStateHandler funnelStateHandler;
	private final ShooterStateHandler shooterStateHandler;

	public Superstructure(String logPath, Robot robot, Supplier<Double> distanceFromTower) {
		this.robot = robot;

		this.currentState = null;

		this.subsystems = Set.of();

		this.targetChecks = new TargetChecks(this);

		this.isSubsystemRunningIndependently = false;
		this.logPath = logPath;

		this.funnelStateHandler = new FunnelStateHandler(robot.getOmni(), robot.getBelly(), logPath, robot.getFunnelDigitalInput());
		this.intakeStateHandler = new IntakeStateHandler();
		this.shooterStateHandler = new ShooterStateHandler(robot.getTurret(), robot.getHood(), robot.getFlyWheel(), distanceFromTower);
	}


	public RobotState getCurrentState() {
		return currentState;
	}

	public boolean isSubsystemRunningIndependently() {
		return isSubsystemRunningIndependently
			|| robot.getIntakeRoller().getCommandsBuilder().isSubsystemRunningIndependently()
			|| robot.getFlyWheel().getCommandBuilder().isSubsystemRunningIndependently()
			|| robot.getBelly().getCommandsBuilder().isSubsystemRunningIndependently()
			|| robot.getHood().getCommandsBuilder().isSubsystemRunningIndependently()
			|| robot.getOmni().getCommandsBuilder().isSubsystemRunningIndependently()
			|| robot.getTurret().getCommandsBuilder().isSubsystemRunningIndependently()
			|| robot.getFourBar().getCommandsBuilder().isSubsystemRunningIndependently();
	}

	public void setIsSubsystemRunningIndependently(boolean isSubsystemRunningIndependently) {
		this.isSubsystemRunningIndependently = isSubsystemRunningIndependently
			|| robot.getIntakeRoller().getCommandsBuilder().isSubsystemRunningIndependently()
			|| robot.getFlyWheel().getCommandBuilder().isSubsystemRunningIndependently()
			|| robot.getBelly().getCommandsBuilder().isSubsystemRunningIndependently()
			|| robot.getHood().getCommandsBuilder().isSubsystemRunningIndependently()
			|| robot.getOmni().getCommandsBuilder().isSubsystemRunningIndependently()
			|| robot.getTurret().getCommandsBuilder().isSubsystemRunningIndependently()
			|| robot.getFourBar().getCommandsBuilder().isSubsystemRunningIndependently();
	}

	public TargetChecks getTargetChecks() {
		return targetChecks;
	}

	public Command setState(RobotState robotState) {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				new InstantCommand(() -> currentState = robotState),
				new InstantCommand(() -> Logger.recordOutput(logPath + "/currentState", robotState)),
				switch (robotState) {
					case STAY_IN_PLACE -> stayInPlace();
					case DRIVE -> idle();
					case INTAKE -> intake();
					case PRE_SHOOT -> preShoot();
					case SHOOT -> shoot();
					case SHOOT_AND_INTAKE -> shootAndIntake();
				}
			),
			currentState
		);
	}

	private Command stayInPlace() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				shooterStateHandler.setState(ShooterState.STAY_IN_PLACE),
				funnelStateHandler.setState(FunnelState.STOP),
				intakeStateHandler.setState(IntakeStateHandler.IntakeState.STAY_IN_PLACE)
			),
			currentState
		);
	}

	private Command idle() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				shooterStateHandler.setState(ShooterState.IDLE),
				funnelStateHandler.setState(FunnelState.DRIVE),
				intakeStateHandler.setState(IntakeStateHandler.IntakeState.CLOSED)
			),
			currentState
		);
	}

	private Command intake() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				shooterStateHandler.setState(ShooterState.IDLE),
				funnelStateHandler.setState(FunnelState.INTAKE),
				intakeStateHandler.setState(IntakeStateHandler.IntakeState.INTAKE)
			),
			currentState
		);
	}

	private Command preShoot() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				shooterStateHandler.setState(ShooterState.SHOOT),
				funnelStateHandler.setState(FunnelState.DRIVE),
				intakeStateHandler.setState(IntakeStateHandler.IntakeState.CLOSED)
			),
			currentState
		);
	}

	private Command shoot() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				shooterStateHandler.setState(ShooterState.SHOOT),
				funnelStateHandler.setState(FunnelState.SHOOT),
				intakeStateHandler.setState(IntakeStateHandler.IntakeState.CLOSED)
			),
			currentState
		);
	}

	private Command shootAndIntake() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				shooterStateHandler.setState(ShooterState.SHOOT),
				funnelStateHandler.setState(FunnelState.SHOOT),
				intakeStateHandler.setState(IntakeStateHandler.IntakeState.INTAKE)
			),
			currentState
		);
	}

	public void log() {
		Logger.recordOutput(logPath + "/IsSubsystemRunningIndependently", isSubsystemRunningIndependently());
	}

	private Command asSubsystemCommand(Command command, RobotState state) {
		return new ParallelCommandGroup(asSubsystemCommand(command, state), new InstantCommand(() -> currentState = state));
	}

}
