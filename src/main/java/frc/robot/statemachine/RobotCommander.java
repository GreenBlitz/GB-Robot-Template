package frc.robot.statemachine;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.statemachine.superstructure.Superstructure;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import java.util.Set;

public class RobotCommander extends GBSubsystem {

	private final Robot robot;
	private final Swerve swerve;
	private final Superstructure superstructure;
	private final PositionTargets positionTargets;

	private RobotState currentState;

	public RobotCommander(String logPath, Robot robot) {
		super(logPath);
		this.robot = robot;
		this.swerve = robot.getSwerve();
		this.positionTargets = new PositionTargets(robot);
		this.superstructure = new Superstructure("StateMachine/Superstructure", robot, () -> robot.getPoseEstimator().getEstimatedPose());
		this.currentState = RobotState.STAY_IN_PLACE;

		setDefaultCommand(
			new ConditionalCommand(
				asSubsystemCommand(Commands.none(), "Disabled"),
				new InstantCommand(
					() -> new DeferredCommand(
						() -> endState(currentState),
						Set.of(
							this,
							swerve,
							robot.getIntakeRoller(),
							robot.getTurret(),
							robot.getFourBar(),
							robot.getBelly(),
							robot.getHood(),
							robot.getOmni(),
							robot.getFlyWheel()
						)
					).schedule()
				),
				this::isSubsystemRunningIndependently
			)

		);
	}

	public RobotState getCurrentState() {
		return currentState;
	}

	public Superstructure getSuperstructure() {
		return superstructure;
	}

	public boolean isSubsystemRunningIndependently() {
		return superstructure.isSubsystemRunningIndependently() || swerve.getCommandsBuilder().isSubsystemRunningIndependently();
	}

	@Override
	protected void subsystemPeriodic() {
		superstructure.periodic();
	}

	public Command driveWith(RobotState state, Command command) {
		Command swerveDriveCommand = swerve.getCommandsBuilder().driveByDriversInputs(state.getSwerveState());
		Command wantedCommand = command.deadlineFor(swerveDriveCommand);
		return asSubsystemCommand(wantedCommand, state);
	}

	public Command driveWith(RobotState state) {
		return driveWith(state, superstructure.setState(state));
	}

	private Command asSubsystemCommand(Command command, RobotState state) {
		return new ParallelCommandGroup(asSubsystemCommand(command, state.name()), new InstantCommand(() -> currentState = state));
	}

	private Command endState(RobotState state) {
		return switch (state) {
			case STAY_IN_PLACE -> driveWith(RobotState.STAY_IN_PLACE);
			case DRIVE, INTAKE, SHOOT, SHOOT_AND_INTAKE -> driveWith(RobotState.DRIVE);
			case PRE_SHOOT -> driveWith(RobotState.PRE_SHOOT);
		};
	}

}
