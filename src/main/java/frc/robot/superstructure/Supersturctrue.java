package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.constants.Field;
import frc.robot.subsystems.elbow.ElbowState;
import frc.robot.subsystems.elbow.ElbowStateHandler;
import frc.robot.subsystems.flywheel.FlywheelState;
import frc.robot.subsystems.flywheel.FlywheelStateHandler;
import frc.robot.subsystems.funnel.FunnelState;
import frc.robot.subsystems.funnel.FunnelStateHandler;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.intake.IntakeStateHandler;
import frc.robot.subsystems.pivot.PivotState;
import frc.robot.subsystems.pivot.PivotStateHandler;
import frc.robot.subsystems.roller.RollerState;
import frc.robot.subsystems.roller.RollerStateHandler;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import org.littletonrobotics.junction.Logger;

public class Supersturctrue {

	private final Robot robot;

	private final Swerve swerve;
	private final ElbowStateHandler elbowStateHandler;
	private final FlywheelStateHandler flywheelStateHandler;
	private final FunnelStateHandler funnelStateHandler;
	private final IntakeStateHandler intakeStateHandler;
	private final PivotStateHandler pivotStateHandler;
	private final RollerStateHandler rollerStateHandler;

	private RobotState currentState;

	public Supersturctrue(Robot robot) {
		this.robot = robot;
		this.swerve = robot.getSwerve();
		this.elbowStateHandler = new ElbowStateHandler(robot.getElbow());
		this.flywheelStateHandler = new FlywheelStateHandler(robot.getFlywheel());
		this.funnelStateHandler = new FunnelStateHandler(robot.getFunnel());
		this.intakeStateHandler = new IntakeStateHandler(robot.getIntake());
		this.pivotStateHandler = new PivotStateHandler(robot.getPivot());
		this.rollerStateHandler = new RollerStateHandler(robot.getRoller());
	}

    public void logStatus() {
        Logger.recordOutput("CurrentState", currentState);
        Logger.recordOutput("isReadyToShoot", isReadyToShoot());
    }

	public RobotState getCurrentState() {
		return currentState;
	}

	public Command setState(RobotState state) {
		this.currentState = state;
		return switch (state) {
			case IDLE -> idle();
			case PRE_SPEAKER -> preSpeaker();
			case PRE_AMP -> preAMP();
			case TRANSFER_SHOOTER_ARM -> null;
			case TRANSFER_ARM_SHOOTER -> null;
			case INTAKE -> intake();
			case SPEAKER -> speaker();
			case AMP -> null;
			case SHOOTER_OUTTAKE -> shooterOuttake();
		};
	}

	//@formatter:off
	private Command idle() {
		return new ParallelCommandGroup(
			rollerStateHandler.setState(RollerState.STOP),
			pivotStateHandler.setState(PivotState.IDLE),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			elbowStateHandler.setState(ElbowState.IDLE),
			intakeStateHandler.setState(IntakeState.STOP),
			funnelStateHandler.setState(FunnelState.STOP),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command preSpeaker() {
		return new ParallelCommandGroup(
			rollerStateHandler.setState(RollerState.STOP),
			funnelStateHandler.setState(FunnelState.STOP),
			intakeStateHandler.setState(IntakeState.STOP),
			pivotStateHandler.setState(PivotState.PRE_SPEAKER),
			flywheelStateHandler.setState(FlywheelState.PRE_SPEAKER),
			elbowStateHandler.setState(ElbowState.IDLE),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER))
		);
	}

	private Command preAMP() {
		return new ParallelCommandGroup(
			rollerStateHandler.setState(RollerState.STOP),
			intakeStateHandler.setState(IntakeState.STOP),
			pivotStateHandler.setState(PivotState.IDLE),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			new SequentialCommandGroup(
				new ParallelDeadlineGroup(
					new RunCommand(() -> {}).until(() -> swerve.isAtHeading(Field.getAngleToAmp())),
					swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP)),
					funnelStateHandler.setState(FunnelState.STOP)
				),
				new ParallelDeadlineGroup(
					elbowStateHandler.setState(ElbowState.PRE_AMP),
					funnelStateHandler.setState(FunnelState.RELEASE_FOR_ARM)
				),
                funnelStateHandler.setState(FunnelState.STOP)
			)
		);
	}

	private Command intake() {
		return new ParallelCommandGroup(
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			pivotStateHandler.setState(PivotState.IDLE),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.NOTE)),
			elbowStateHandler.setState(ElbowState.INTAKE),
			new SequentialCommandGroup(
				new ParallelDeadlineGroup(
					intakeStateHandler.setState(IntakeState.INTAKE).until(() -> robot.getIntake().isObjectIn()),
					rollerStateHandler.setState(RollerState.IN)
				),
				new ParallelCommandGroup(
					intakeStateHandler.setState(IntakeState.INTAKE_WITH_FUNNEL),
					funnelStateHandler.setState(FunnelState.INTAKE),
					rollerStateHandler.setState(RollerState.IN)
				).until(() -> robot.getFunnel().isObjectIn())
			)
		);
	}

	private Command shooterOuttake() {
		return new ParallelCommandGroup(
			rollerStateHandler.setState(RollerState.OUT),
			elbowStateHandler.setState(ElbowState.MANUAL),
			intakeStateHandler.setState(IntakeState.OUTTAKE),
			funnelStateHandler.setState(FunnelState.OUTTAKE),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			pivotStateHandler.setState(PivotState.INTAKE)
		);
	}

	private boolean isReadyToShoot() {
		boolean isPivotReady = robot.getPivot().isAtPosition(PivotState.PRE_SPEAKER.getTargetPosition(), Rotation2d.fromDegrees(4));

		boolean isFlywheelReady = robot.getFlywheel()
			.isAtVelocities(
				FlywheelState.PRE_SPEAKER.getRightVelocity(),
				FlywheelState.PRE_SPEAKER.getLeftVelocity(),
				Rotation2d.fromRotations(4)
			);
        Logger.recordOutput("isPivotReady", isPivotReady);
        Logger.recordOutput("isFlywheelReady", isFlywheelReady);
		return isFlywheelReady && isPivotReady;
	}

	private Command speaker() {
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(
                funnelStateHandler.setState(FunnelState.STOP).until(() -> true),
                new RunCommand(() -> {}, robot.getFunnel()).until(this::isReadyToShoot),
                funnelStateHandler.setState(FunnelState.SHOOT).until(() -> !robot.getFunnel().isObjectIn())
            ),
			intakeStateHandler.setState(IntakeState.STOP),
			pivotStateHandler.setState(PivotState.PRE_SPEAKER),
			flywheelStateHandler.setState(FlywheelState.PRE_SPEAKER),
			elbowStateHandler.setState(ElbowState.IDLE),
			rollerStateHandler.setState(RollerState.STOP),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER))
		).andThen(idle());
	}


	private Command transferShooterArm() {
		return new SequentialCommandGroup(
			new ParallelCommandGroup(
					pivotStateHandler.setState(PivotState.TRANSFER),
					elbowStateHandler.setState(ElbowState.TRANSFER)
			),
			funnelStateHandler.setState(FunnelState.TRANSFER_TO_ARM)
			// add roller
		);
	}

	private Command transferArmShooter() {
		return new SequentialCommandGroup(
				new ParallelCommandGroup(
						pivotStateHandler.setState(PivotState.TRANSFER),
						elbowStateHandler.setState(ElbowState.TRANSFER)
				),
				funnelStateHandler.setState(FunnelState.TRANSFER_TO_SHOOTER)
				// add roller
		);
	}
	//@formatter:on

}
