package frc.robot.statemachine.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.statemachine.RobotState;
import frc.robot.statemachine.funnelstatehandler.FunnelState;
import frc.robot.statemachine.funnelstatehandler.FunnelStateHandler;
import frc.robot.statemachine.intakestatehandler.IntakeState;
import frc.robot.statemachine.intakestatehandler.IntakeStateHandler;
import frc.robot.statemachine.shooterstatehandler.ShooterState;
import frc.robot.statemachine.shooterstatehandler.ShooterStateHandler;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class Superstructure {

	private final Robot robot;
	private final TargetChecks targetChecks;
	private boolean isSubsystemRunningIndependently;
	private final String logPath;

	private RobotState currentState;

	private final IntakeStateHandler intakeStateHandler;
	private final FunnelStateHandler funnelStateHandler;
	private final ShooterStateHandler shooterStateHandler;

	public Superstructure(String logPath, Robot robot, Supplier<Pose2d> robotPoseSupplier) {
		this.robot = robot;
		this.logPath = logPath;

		this.funnelStateHandler = new FunnelStateHandler(robot.getOmni(), robot.getBelly(), logPath, robot.getFunnelDigitalInput());
		this.intakeStateHandler = new IntakeStateHandler(robot.getFourBar(), robot.getIntakeRoller(), robot.getIntakeRollerSensor(), logPath);
		this.shooterStateHandler = new ShooterStateHandler(robot.getTurret(), robot.getHood(), robot.getFlyWheel(), robotPoseSupplier, logPath);

		this.targetChecks = new TargetChecks(this);

		this.currentState = RobotState.STAY_IN_PLACE;
		this.isSubsystemRunningIndependently = false;
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
		this.isSubsystemRunningIndependently = isSubsystemRunningIndependently;
	}

	public TargetChecks getTargetChecks() {
		return targetChecks;
	}

	public Command setState(RobotState robotState) {
		return new ParallelCommandGroup(
			new InstantCommand(() -> currentState = robotState),
			new InstantCommand(() -> Logger.recordOutput(logPath + "/CurrentState", robotState)),
			switch (robotState) {
				case STAY_IN_PLACE -> stayInPlace();
				case DRIVE -> idle();
				case INTAKE -> intake();
				case PRE_SHOOT -> preShoot();
				case SHOOT -> shoot();
				case SHOOT_AND_INTAKE -> shootAndIntake();
			}
		);
	}

	private Command stayInPlace() {
		return new ParallelCommandGroup(
			shooterStateHandler.setState(ShooterState.STAY_IN_PLACE),
			funnelStateHandler.setState(FunnelState.STOP),
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
		funnelStateHandler.periodic();
		intakeStateHandler.periodic();
		Logger.recordOutput(logPath + "/IsSubsystemRunningIndependently", isSubsystemRunningIndependently());
	}

}
