package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
<<<<<<< HEAD
import frc.robot.subsystems.flywheel.FlywheelState;
import frc.robot.subsystems.flywheel.FlywheelStateHandler;
=======
import frc.robot.constants.Field;
import frc.robot.subsystems.funnel.FunnelState;
import frc.robot.subsystems.funnel.FunnelStateHandler;
>>>>>>> flywheel
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

	private final Robot robot;
	private final Swerve swerve;
<<<<<<< HEAD
	private final FlywheelStateHandler flywheelStateHandler;
=======
	private final FunnelStateHandler funnelStateHandler;
>>>>>>> flywheel

	private RobotState currentState;

	public Superstructure(Robot robot) {
		this.robot = robot;
		this.swerve = robot.getSwerve();
<<<<<<< HEAD
		this.flywheelStateHandler = new FlywheelStateHandler(robot.getFlywheel());
=======
		this.funnelStateHandler = new FunnelStateHandler(robot);
>>>>>>> flywheel
	}

	public RobotState getCurrentState() {
		return currentState;
	}

	public void logStatus() {
		Logger.recordOutput("CurrentState", currentState);
	}

	private boolean isNoteInShooter() {
		return robot.getFunnel().isNoteInShooter();
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
			case INTAKE_OUTTAKE -> intakeOuttake();
			case SHOOTER_OUTTAKE -> shooterOuttake();
		};
	}

	//@formatter:off
	public Command idle() {
		return new ParallelCommandGroup(
<<<<<<< HEAD
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
				flywheelStateHandler.setState(FlywheelState.DEFAULT)
=======
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
			funnelStateHandler.setState(FunnelState.STOP)
>>>>>>> flywheel
		);
	}

	public Command intake() {
		return new ParallelCommandGroup(
<<<<<<< HEAD
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.NOTE)),
				flywheelStateHandler.setState(FlywheelState.DEFAULT)
=======
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.NOTE)),
			funnelStateHandler.setState(FunnelState.NOTE_TO_SHOOTER)
>>>>>>> flywheel
		);
	}

	public Command preSpeaker() {
		return new ParallelCommandGroup(
<<<<<<< HEAD
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER)),
				flywheelStateHandler.setState(FlywheelState.SHOOTING)

=======
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER)),
			funnelStateHandler.setState(FunnelState.STOP)
>>>>>>> flywheel
		);
	}

	public Command speaker() {
		return new ParallelCommandGroup(
<<<<<<< HEAD
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER)),
				flywheelStateHandler.setState(FlywheelState.SHOOTING)

=======
			funnelStateHandler.setState(FunnelState.SPEAKER),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER))
>>>>>>> flywheel
		);
	}

	public Command preAmp() {
		return new ParallelCommandGroup(
<<<<<<< HEAD
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP)),
				flywheelStateHandler.setState(FlywheelState.DEFAULT)
=======
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP)),
			funnelStateHandler.setState(FunnelState.STOP)
>>>>>>> flywheel
		);
	}

	public Command amp() {
		return new ParallelCommandGroup(
<<<<<<< HEAD
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP)),
				flywheelStateHandler.setState(FlywheelState.DEFAULT)
=======
			swerve.getCommandsBuilder()
				.saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP))
				.until(() -> swerve.isAtHeading(Field.getAngleToAmp())),
			funnelStateHandler.setState(FunnelState.AMP)
>>>>>>> flywheel
		);
	}

	public Command transferShooterElevator() {
		return new ParallelCommandGroup(
<<<<<<< HEAD
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
				flywheelStateHandler.setState(FlywheelState.DEFAULT)
=======
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
			funnelStateHandler.setState(FunnelState.SHOOTER_TO_ELEVATOR)
>>>>>>> flywheel
		);
	}

	public Command transferElevatorShooter() {
		return new ParallelCommandGroup(
<<<<<<< HEAD
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
				flywheelStateHandler.setState(FlywheelState.DEFAULT)

=======
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
			funnelStateHandler.setState(FunnelState.NOTE_TO_SHOOTER)
>>>>>>> flywheel
		);
	}

	public Command intakeOuttake() {
		return new ParallelCommandGroup(
<<<<<<< HEAD
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
				flywheelStateHandler.setState(FlywheelState.DEFAULT)

=======
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
			funnelStateHandler.setState(FunnelState.INTAKE_OUTTAKE)
>>>>>>> flywheel
		);
	}

	public Command shooterOuttake() {
		return new ParallelCommandGroup(
<<<<<<< HEAD
				swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
				flywheelStateHandler.setState(FlywheelState.SHOOTER_OUTTAKE)
		);
=======
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
			funnelStateHandler.setState(FunnelState.SHOOTER_OUTTAKE)
		).until(() -> !isNoteInShooter());
>>>>>>> flywheel
	}
	//@formatter:on

}
