package frc.robot.superstructure;

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
import frc.robot.subsystems.wrist.WristState;
import frc.robot.subsystems.wrist.WristStateHandler;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

	private final Robot robot;

	private final Swerve swerve;
	private final ElbowStateHandler elbowStateHandler;
	private final FlywheelStateHandler flywheelStateHandler;
	private final FunnelStateHandler funnelStateHandler;
	private final IntakeStateHandler intakeStateHandler;
	private final PivotStateHandler pivotStateHandler;
	private final RollerStateHandler rollerStateHandler;
	private final WristStateHandler wristStateHandler;

	private RobotState currentState;

	public Superstructure(Robot robot) {
		this.robot = robot;
		this.swerve = robot.getSwerve();
		this.elbowStateHandler = new ElbowStateHandler(robot.getElbow());
		this.flywheelStateHandler = new FlywheelStateHandler(robot.getFlywheel());
		this.funnelStateHandler = new FunnelStateHandler(robot.getFunnel());
		this.intakeStateHandler = new IntakeStateHandler(robot.getIntake());
		this.pivotStateHandler = new PivotStateHandler(robot.getPivot());
		this.rollerStateHandler = new RollerStateHandler(robot.getRoller());
		this.wristStateHandler = new WristStateHandler(robot.getWrist());
	}

	public RobotState getCurrentState() {
		return currentState;
	}

	public void logStatus() {
		Logger.recordOutput("CurrentState", currentState);
	}

	private boolean isObjectInRoller() {
		return robot.getRoller().isObjectIn();
	}

	private boolean isObjectInIntake() {
		return robot.getIntake().isObjectIn();
	}

	private boolean isObjectInFunnel() {
		return robot.getFunnel().isObjectIn();
	}

	private boolean isReadyToTransfer() {
		boolean isPivotReady = robot.getPivot().isAtPosition(PivotState.TRANSFER.getTargetPosition(), Tolerances.PIVOT_POSITION);
		boolean isElbowReady = robot.getElbow().isAtAngle(ElbowState.TRANSFER.getTargetPosition(), Tolerances.ELBOW_POSITION);

		return isElbowReady && isPivotReady;
	}

	private boolean isReadyToShoot() {
		boolean isPivotReady = robot.getPivot().isAtPosition(PivotState.PRE_SPEAKER.getTargetPosition(), Tolerances.PIVOT_POSITION);

		boolean isFlywheelReady = robot.getFlywheel()
			.isAtVelocities(
				FlywheelState.PRE_SPEAKER.getRightVelocity(),
				FlywheelState.PRE_SPEAKER.getLeftVelocity(),
				Tolerances.FLYWHEEL_VELOCITY_PER_SECOND
			);

		return isFlywheelReady && isPivotReady;
	}


	public Command setState(RobotState state) {
		this.currentState = state;
		return switch (state) {
			case IDLE -> idle();
			case SHOOTER_INTAKE -> shooterIntake();
			case ARM_INTAKE -> armIntake();
			case PRE_SPEAKER -> preSpeaker();
			case SPEAKER -> speaker();
			case PRE_AMP -> preAMP();
			case AMP -> amp();
			case TRANSFER_SHOOTER_TO_ARM -> transferShooterToArm();
			case TRANSFER_ARM_TO_SHOOTER -> transferArmToShooter();
			case SHOOTER_OUTTAKE -> shooterOuttake();
			case ARM_OUTTAKE -> armOuttake();
		};
	}

	//@formatter:off
	private Command idle() {
		return new ParallelCommandGroup(
			rollerStateHandler.setState(RollerState.MANUAL),
			intakeStateHandler.setState(IntakeState.STOP),
			funnelStateHandler.setState(FunnelState.MANUAL),
			pivotStateHandler.setState(PivotState.IDLE),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			elbowStateHandler.setState(ElbowState.IDLE),
			wristStateHandler.setState(WristState.IN_ARM),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command shooterIntake() {
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					intakeStateHandler.setState(IntakeState.INTAKE),
					rollerStateHandler.setState(RollerState.ROLL_IN),
					funnelStateHandler.setState(FunnelState.STOP)
				).until(this::isObjectInIntake),
				new ParallelCommandGroup(
					intakeStateHandler.setState(IntakeState.INTAKE_WITH_FUNNEL),
					funnelStateHandler.setState(FunnelState.INTAKE),
					rollerStateHandler.setState(RollerState.ROLL_IN)
				).withTimeout(0.475),//.until(this::isObjectInFunnel)
				new ParallelCommandGroup(
					intakeStateHandler.setState(IntakeState.STOP),
					rollerStateHandler.setState(RollerState.STOP),
					funnelStateHandler.setState(FunnelState.STOP)
				)
			),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			pivotStateHandler.setState(PivotState.IDLE),
			elbowStateHandler.setState(ElbowState.SHOOTER_INTAKE),
			wristStateHandler.setState(WristState.IN_ARM),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.NOTE))
		);
	}

	private Command armIntake() {
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					intakeStateHandler.setState(IntakeState.INTAKE),
					rollerStateHandler.setState(RollerState.ROLL_IN),
					wristStateHandler.setState(WristState.ARM_INTAKE)
				).until(this::isObjectInIntake),
				new ParallelCommandGroup(
					intakeStateHandler.setState(IntakeState.INTAKE_WITH_ARM),
					rollerStateHandler.setState(RollerState.ROLL_IN),
					wristStateHandler.setState(WristState.ARM_INTAKE)
				).withTimeout(0.3),////.until(this::isObjectInRoller)
				new ParallelCommandGroup(
					intakeStateHandler.setState(IntakeState.INTAKE_WITH_ARM),
					rollerStateHandler.setState(RollerState.STOP),
					wristStateHandler.setState(WristState.IN_ARM)
				).until(() -> robot.getWrist().isAtPosition(WristState.IN_ARM.getPosition(), Tolerances.WRIST_POSITION)),
				new ParallelCommandGroup(
					intakeStateHandler.setState(IntakeState.STOP),
					rollerStateHandler.setState(RollerState.STOP),
					wristStateHandler.setState(WristState.IN_ARM)
				)
			),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			pivotStateHandler.setState(PivotState.IDLE),
			elbowStateHandler.setState(ElbowState.ARM_INTAKE),
			funnelStateHandler.setState(FunnelState.STOP),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.NOTE))
		);
	}

	private Command preSpeaker() {
		return new ParallelCommandGroup(
			rollerStateHandler.setState(RollerState.STOP),
			intakeStateHandler.setState(IntakeState.STOP),
			funnelStateHandler.setState(FunnelState.STOP),
			pivotStateHandler.setState(PivotState.PRE_SPEAKER),
			flywheelStateHandler.setState(FlywheelState.PRE_SPEAKER),
			elbowStateHandler.setState(ElbowState.IDLE),
			wristStateHandler.setState(WristState.IN_ARM),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER))
		);
	}

	private Command speaker() {
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				funnelStateHandler.setState(FunnelState.STOP).until(this::isReadyToShoot),
				funnelStateHandler.setState(FunnelState.SHOOT).withTimeout(3),// .until(() -> !isObjectInFunnel())
				funnelStateHandler.setState(FunnelState.STOP)
			),
			rollerStateHandler.setState(RollerState.STOP),
			intakeStateHandler.setState(IntakeState.STOP),
			pivotStateHandler.setState(PivotState.PRE_SPEAKER),
			flywheelStateHandler.setState(FlywheelState.PRE_SPEAKER),
			elbowStateHandler.setState(ElbowState.IDLE),
			wristStateHandler.setState(WristState.IN_ARM),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER))
		);
	}

	private Command preAMP() {
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP)),
					funnelStateHandler.setState(FunnelState.STOP)
				).until(() -> swerve.isAtHeading(Field.getAngleToAmp())),
				new ParallelCommandGroup(
					elbowStateHandler.setState(ElbowState.PRE_AMP),
					funnelStateHandler.setState(FunnelState.RELEASE_FOR_ARM),
					intakeStateHandler.setState(IntakeState.RELEASE_FOR_ARM)
				).until(() -> robot.getElbow().isAtAngle(ElbowState.PRE_AMP.getTargetPosition(), Tolerances.ELBOW_POSITION)),
				intakeStateHandler.setState(IntakeState.STOP),
				funnelStateHandler.setState(FunnelState.STOP)
			),
			rollerStateHandler.setState(RollerState.STOP),
			pivotStateHandler.setState(PivotState.IDLE),
			wristStateHandler.setState(WristState.IN_ARM),
			flywheelStateHandler.setState(FlywheelState.DEFAULT)
		);
	}

	private Command amp() {
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP),
					rollerStateHandler.setState(RollerState.STOP),
					swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP))
				).until(() -> swerve.isAtHeading(Field.getAngleToAmp())),
				new ParallelCommandGroup(
					elbowStateHandler.setState(ElbowState.PRE_AMP),
					funnelStateHandler.setState(FunnelState.RELEASE_FOR_ARM),
					intakeStateHandler.setState(IntakeState.RELEASE_FOR_ARM)
				).until(() -> robot.getElbow().isAtAngle(ElbowState.PRE_AMP.getTargetPosition(), Tolerances.ELBOW_POSITION)),
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP),
					rollerStateHandler.setState(RollerState.ROLL_OUT)
				).withTimeout(3),//.until(() -> !isObjectInRoller())
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP),
					rollerStateHandler.setState(RollerState.STOP),
					intakeStateHandler.setState(IntakeState.STOP),
				)
			),
			pivotStateHandler.setState(PivotState.IDLE),
			wristStateHandler.setState(WristState.IN_ARM),
			flywheelStateHandler.setState(FlywheelState.DEFAULT)
		);
	}

	private Command transferShooterToArm() {
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					pivotStateHandler.setState(PivotState.TRANSFER),
					elbowStateHandler.setState(ElbowState.TRANSFER),
					rollerStateHandler.setState(RollerState.STOP),
					funnelStateHandler.setState(FunnelState.STOP)
				).until(this::isReadyToTransfer),
				new ParallelCommandGroup(
					rollerStateHandler.setState(RollerState.ROLL_IN),
					funnelStateHandler.setState(FunnelState.TRANSFER_TO_ARM)
				).withTimeout(0.6),//.until(this::isObjectInRoller),
				new ParallelDeadlineGroup(
					rollerStateHandler.setState(RollerState.AFTER_INTAKE),
					funnelStateHandler.setState(FunnelState.TRANSFER_TO_ARM)
				),
				new ParallelCommandGroup(
					rollerStateHandler.setState(RollerState.STOP),
					funnelStateHandler.setState(FunnelState.STOP),
					pivotStateHandler.setState(PivotState.IDLE),
					elbowStateHandler.setState(ElbowState.IDLE)
				)
			),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			intakeStateHandler.setState(IntakeState.STOP),
			wristStateHandler.setState(WristState.IN_ARM),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command transferArmToShooter() {
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					pivotStateHandler.setState(PivotState.TRANSFER),
					elbowStateHandler.setState(ElbowState.TRANSFER),
					rollerStateHandler.setState(RollerState.STOP),
					intakeStateHandler.setState(IntakeState.STOP),
					funnelStateHandler.setState(FunnelState.STOP)
				).until(this::isReadyToTransfer),
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.INTAKE),
					intakeStateHandler.setState(IntakeState.INTAKE_WITH_FUNNEL),
					rollerStateHandler.setState(RollerState.ROLL_OUT)
				).withTimeout(0.7),//.until(this::isObjectInFunnel)
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP),
					intakeStateHandler.setState(IntakeState.STOP),
					rollerStateHandler.setState(RollerState.STOP),
					pivotStateHandler.setState(PivotState.IDLE),
					elbowStateHandler.setState(ElbowState.IDLE)
				)
			),
			wristStateHandler.setState(WristState.IN_ARM),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command shooterOuttake() {
		return new ParallelCommandGroup(
			rollerStateHandler.setState(RollerState.ROLL_OUT),
			intakeStateHandler.setState(IntakeState.OUTTAKE),
			funnelStateHandler.setState(FunnelState.OUTTAKE),
			pivotStateHandler.setState(PivotState.INTAKE),
			elbowStateHandler.setState(ElbowState.MANUAL),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			wristStateHandler.setState(WristState.IN_ARM),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command armOuttake() {
		return new ParallelCommandGroup(
			rollerStateHandler.setState(RollerState.ROLL_IN),
			elbowStateHandler.setState(ElbowState.ARM_INTAKE),
			wristStateHandler.setState(WristState.IN_ARM),
			intakeStateHandler.setState(IntakeState.STOP),
			funnelStateHandler.setState(FunnelState.STOP),
			pivotStateHandler.setState(PivotState.INTAKE),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE)
		);
	}
	//@formatter:on

}
