package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

	private final Robot robot;
	private final Swerve swerve;

	private RobotState currentState;

	public Superstructure(Robot robot) {
		this.robot = robot;
		this.swerve = robot.getSwerve();
	}

	public RobotState getCurrentState() {
		return currentState;
	}

	public void logStatus() {
		Logger.recordOutput("CurrentState", currentState);
	}

	public Command setState(RobotState state) {
		return switch (state) {
			case IDLE -> idle();
			case INTAKE -> intake();
			case PRE_SPEAKER -> preSpeaker();
			case SPEAKER -> speaker();
			case PRE_AMP -> preAmp();
			case AMP -> amp();
			case TRANSFER_SHOOTER_ELEVATOR -> transferShooterElevator();
			case TRANSFER_ELEVATOR_SHOOTER -> transferElevatorShooter();
			case SHOOTER_OUTTAKE -> shooterOuttake();
		};
	}

	//@formatter:off
	public Command idle() {
		return new ParallelCommandGroup(
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE)
		);
	}

	public Command intake() {
		return new ParallelCommandGroup(
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.NOTE))
		);
	}

	public Command preSpeaker() {
		return new ParallelCommandGroup(
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER))
		);
	}

	public Command speaker() {
		return new ParallelCommandGroup(
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER))
		);
	}

	public Command preAmp() {
		return new ParallelCommandGroup(
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP))
		);
	}

	public Command amp() {
		return new ParallelCommandGroup(
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP))
		);
	}

	public Command transferShooterElevator() {
		return new ParallelCommandGroup(
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE)
		);
	}

	public Command transferElevatorShooter() {
		return new ParallelCommandGroup(
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE)
		);
	}

	public Command shooterOuttake() {
		return new ParallelCommandGroup(
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE)
		);
	}
	//@formatter:on

}
