package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.JoysticksBindings;
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
import frc.robot.subsystems.pivot.PivotSpeakerInterpolationMap;
import frc.robot.subsystems.pivot.PivotState;
import frc.robot.subsystems.pivot.PivotStateHandler;
import frc.robot.subsystems.roller.RollerState;
import frc.robot.subsystems.roller.RollerStateHandler;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import frc.robot.subsystems.wrist.WristState;
import frc.robot.subsystems.wrist.WristStateHandler;
import frc.utils.joysticks.SmartJoystick;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class Superstructure {

	private static final double NOTE_IN_RUMBLE_POWER = 0.5;

	private final String logPath;
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
	private boolean enableChangeStateAutomatically;

	public Superstructure(String logPath, Robot robot) {
		this.robot = robot;
		this.swerve = robot.getSwerve();
		this.elbowStateHandler = new ElbowStateHandler(robot.getElbow());
		this.flywheelStateHandler = new FlywheelStateHandler(robot.getFlywheel());
		this.funnelStateHandler = new FunnelStateHandler(robot.getFunnel());
		this.intakeStateHandler = new IntakeStateHandler(robot.getIntake());
		this.pivotStateHandler = new PivotStateHandler(
				robot.getPivot(), Optional.of(() -> robot.getPoseEstimator().getEstimatedPose()));
		this.rollerStateHandler = new RollerStateHandler(robot.getRoller());
		this.wristStateHandler = new WristStateHandler(robot.getWrist());

		this.logPath = logPath;
		this.enableChangeStateAutomatically = true;
	}

	public RobotState getCurrentState() {
		return currentState;
	}

	public boolean isEnableChangeStateAutomatically() {
		return enableChangeStateAutomatically;
	}

	public void logStatus() {
		Logger.recordOutput(logPath + "CurrentState", currentState);
		Logger.recordOutput(logPath + "EnableChangeStateAutomatically", enableChangeStateAutomatically);
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
		boolean isElbowReady = robot.getElbow().isAtAngle(ElbowState.TRANSFER.getTargetPosition(), Tolerances.ELBOW_POSITION_TRANSFER);

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

	private boolean isReadyToShootInterpolation() {
		double metersFromSpeaker = Field.getSpeaker()
			.toTranslation2d()
			.getDistance(robot.getPoseEstimator().getEstimatedPose().getTranslation());
		boolean isPivotReady = robot.getPivot()
			.isAtPosition(
				Rotation2d.fromRadians(PivotSpeakerInterpolationMap.METERS_TO_RADIANS.get(metersFromSpeaker)),
				Tolerances.PIVOT_POSITION
			);

		boolean isFlywheelReady = robot.getFlywheel()
			.isAtVelocities(
				FlywheelState.PRE_SPEAKER.getRightVelocity(),
				FlywheelState.PRE_SPEAKER.getLeftVelocity(),
				Tolerances.FLYWHEEL_VELOCITY_PER_SECOND
			);

		return isFlywheelReady && isPivotReady;
	}

	private Command setCurrentStateName(RobotState state) {
		return new InstantCommand(() -> currentState = state);
	}

	private Command enableChangeStateAutomatically(boolean enable) {
		return new InstantCommand(() -> enableChangeStateAutomatically = enable);
	}

	private Command noteInRumble() {
		SmartJoystick mainJoystick = JoysticksBindings.getMainJoystick();
		return new FunctionalCommand(
			() -> {},
			() -> mainJoystick.setRumble(GenericHID.RumbleType.kBothRumble, NOTE_IN_RUMBLE_POWER),
			interrupted -> mainJoystick.stopRumble(GenericHID.RumbleType.kBothRumble),
			() -> false
		).withTimeout(Timeouts.NOTE_IN_RUMBLE);
	}

	public Command setState(RobotState state) {
		return switch (state) {
			case IDLE -> idle();
			case INTAKE -> intake();
			case ARM_INTAKE -> armIntake();
			case PRE_SPEAKER -> preSpeaker();
			case SPEAKER -> speaker();
			case PRE_AMP -> preAMP();
			case AMP -> amp();
			case TRANSFER_SHOOTER_TO_ARM -> transferShooterToArm();
			case TRANSFER_ARM_TO_SHOOTER -> transferArmToShooter();
			case INTAKE_OUTTAKE -> intakeOuttake();
			case ARM_OUTTAKE -> armOuttake();
			case INTAKE_WITH_FLYWHEEL -> intakeWithFlywheel();
		};
	}

	//@formatter:off
	private Command idle() {
		return new ParallelCommandGroup(
			enableChangeStateAutomatically(true),
			setCurrentStateName(RobotState.IDLE),
			rollerStateHandler.setState(RollerState.MANUAL),
			intakeStateHandler.setState(IntakeState.STOP),
			funnelStateHandler.setState(FunnelState.MANUAL),
			pivotStateHandler.setState(PivotState.IDLE),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			elbowStateHandler.setState(ElbowState.IDLE),
			wristStateHandler.setState(WristState.DEFAULT),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE)
		).handleInterrupt(() -> enableChangeStateAutomatically(true).schedule());
	}

	private Command intake() {
		return new ParallelCommandGroup(
			setCurrentStateName(RobotState.INTAKE),
			new SequentialCommandGroup(
				enableChangeStateAutomatically(false),
				new ParallelCommandGroup(
					intakeStateHandler.setState(IntakeState.INTAKE),
					rollerStateHandler.setState(RollerState.ROLL_IN),
					funnelStateHandler.setState(FunnelState.STOP)
				).until(this::isObjectInIntake),
				new ParallelCommandGroup(
					noteInRumble(),
					intakeStateHandler.setState(IntakeState.INTAKE_WITH_FUNNEL),
					funnelStateHandler.setState(FunnelState.INTAKE),
					rollerStateHandler.setState(RollerState.ROLL_IN)
				).withTimeout(Timeouts.INTAKE_FUNNEL_SECONDS),//.until(this::isObjectInFunnel)
				enableChangeStateAutomatically(true),
				new ParallelCommandGroup(
					intakeStateHandler.setState(IntakeState.STOP),
					rollerStateHandler.setState(RollerState.STOP),
					funnelStateHandler.setState(FunnelState.STOP)
				)
			),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			pivotStateHandler.setState(PivotState.IDLE),
			elbowStateHandler.setState(ElbowState.INTAKE),
			wristStateHandler.setState(WristState.IN_ARM),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.NOTE))
		).handleInterrupt(() -> enableChangeStateAutomatically(true).schedule());
	}

	private Command armIntake() {
		return new ParallelCommandGroup(
			setCurrentStateName(RobotState.ARM_INTAKE),
			new SequentialCommandGroup(
				enableChangeStateAutomatically(false),
				new ParallelCommandGroup(
					intakeStateHandler.setState(IntakeState.INTAKE),
					rollerStateHandler.setState(RollerState.ROLL_IN),
					wristStateHandler.setState(WristState.ARM_INTAKE)
				).until(this::isObjectInIntake),
				new ParallelCommandGroup(
					noteInRumble(),
					intakeStateHandler.setState(IntakeState.INTAKE_WITH_ARM),
					rollerStateHandler.setState(RollerState.ROLL_IN),
					funnelStateHandler.setState(FunnelState.SLOW_INTAKE),
					wristStateHandler.setState(WristState.ARM_INTAKE)
				).withTimeout(Timeouts.INTAKE_ROLLER_SECONDS),//.until(this::isObjectInRoller)
				new ParallelCommandGroup(
					intakeStateHandler.setState(IntakeState.INTAKE_WITH_ARM),
					rollerStateHandler.setState(RollerState.STOP),
					funnelStateHandler.setState(FunnelState.SLOW_INTAKE),
					wristStateHandler.setState(WristState.DEFAULT)
				).withTimeout(Timeouts.WRIST_TO_POSITION_SECONDS)
				 .until(() -> robot.getWrist().isAtPosition(WristState.DEFAULT.getPosition(), Tolerances.WRIST_POSITION)),
				enableChangeStateAutomatically(true),
				new ParallelCommandGroup(
					intakeStateHandler.setState(IntakeState.STOP),
					rollerStateHandler.setState(RollerState.STOP),
					funnelStateHandler.setState(FunnelState.STOP),
					wristStateHandler.setState(WristState.DEFAULT)
				)
			),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			pivotStateHandler.setState(PivotState.ARM_INTAKE),
			elbowStateHandler.setState(ElbowState.ARM_INTAKE),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.NOTE))
		).handleInterrupt(() -> enableChangeStateAutomatically(true).schedule());
	}

	private Command preSpeaker() {
		return new ParallelCommandGroup(
			enableChangeStateAutomatically(true),
			setCurrentStateName(RobotState.PRE_SPEAKER),
			rollerStateHandler.setState(RollerState.STOP),
			intakeStateHandler.setState(IntakeState.STOP),
			funnelStateHandler.setState(FunnelState.MANUAL),
			pivotStateHandler.setState(PivotState.INTERPOLATE),
			flywheelStateHandler.setState(FlywheelState.PRE_SPEAKER),
			elbowStateHandler.setState(ElbowState.IDLE),
			wristStateHandler.setState(WristState.IN_ARM),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER))
		).handleInterrupt(() -> enableChangeStateAutomatically(true).schedule());
	}

	private Command speaker() {
		return new ParallelCommandGroup(
			setCurrentStateName(RobotState.SPEAKER),
			new SequentialCommandGroup(
				enableChangeStateAutomatically(false),
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP),
					intakeStateHandler.setState(IntakeState.STOP)
				).until(this::isReadyToShootInterpolation),
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.SHOOT),
					intakeStateHandler.setState(IntakeState.INTAKE_WITH_FUNNEL)
				).withTimeout(Timeouts.SHOOTING_SECONDS),// .until(() -> !isObjectInFunnel()),
				enableChangeStateAutomatically(true),
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP),
					intakeStateHandler.setState(IntakeState.STOP)
				)
			),
			rollerStateHandler.setState(RollerState.STOP),
			pivotStateHandler.setState(PivotState.INTERPOLATE),
			flywheelStateHandler.setState(FlywheelState.PRE_SPEAKER),
			elbowStateHandler.setState(ElbowState.INTAKE),
			wristStateHandler.setState(WristState.IN_ARM),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER))
		).handleInterrupt(() -> enableChangeStateAutomatically(true).schedule());
	}

	private Command preAMP() {
		return new ParallelCommandGroup(
			enableChangeStateAutomatically(false),
			setCurrentStateName(RobotState.PRE_AMP),
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
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP),
					intakeStateHandler.setState(IntakeState.STOP)
				)
			),
			rollerStateHandler.setState(RollerState.STOP),
			pivotStateHandler.setState(PivotState.IDLE),
			wristStateHandler.setState(WristState.IN_ARM),
			flywheelStateHandler.setState(FlywheelState.DEFAULT)
		).handleInterrupt(() -> enableChangeStateAutomatically(true).schedule());
	}

	private Command amp() {
		return new ParallelCommandGroup(
			setCurrentStateName(RobotState.AMP),
			new SequentialCommandGroup(
				enableChangeStateAutomatically(false),
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
					intakeStateHandler.setState(IntakeState.STOP),
					rollerStateHandler.setState(RollerState.ROLL_OUT)
				).withTimeout(Timeouts.AMP_RELEASE_SECONDS),//.until(() -> !isObjectInRoller())
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP),
					rollerStateHandler.setState(RollerState.STOP),
					intakeStateHandler.setState(IntakeState.STOP)
				)
			),
			pivotStateHandler.setState(PivotState.IDLE),
			wristStateHandler.setState(WristState.IN_ARM),
			flywheelStateHandler.setState(FlywheelState.DEFAULT)
		).handleInterrupt(() -> enableChangeStateAutomatically(true).schedule());
	}

	private Command transferShooterToArm() {
		return new ParallelCommandGroup(
			setCurrentStateName(RobotState.TRANSFER_SHOOTER_TO_ARM),
			new SequentialCommandGroup(
				enableChangeStateAutomatically(false),
				new ParallelCommandGroup(
					pivotStateHandler.setState(PivotState.TRANSFER),
					elbowStateHandler.setState(ElbowState.TRANSFER),
					rollerStateHandler.setState(RollerState.STOP),
					funnelStateHandler.setState(FunnelState.STOP)
				).until(this::isReadyToTransfer),
				new ParallelCommandGroup(
					rollerStateHandler.setState(RollerState.ROLL_IN),
					funnelStateHandler.setState(FunnelState.TRANSFER_TO_ARM)
				).withTimeout(Timeouts.TRANSFER_SHOOTER_ARM_SECONDS),//.until(this::isObjectInRoller),
				new ParallelDeadlineGroup(
					rollerStateHandler.setState(RollerState.AFTER_INTAKE),
					funnelStateHandler.setState(FunnelState.TRANSFER_TO_ARM)
				).withTimeout(Timeouts.INTAKE_ARM_1_ROTATION_SECONDS),
				enableChangeStateAutomatically(true),
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
		).handleInterrupt(() -> enableChangeStateAutomatically(true).schedule());
	}

	private Command transferArmToShooter() {
		return new ParallelCommandGroup(
			setCurrentStateName(RobotState.TRANSFER_ARM_TO_SHOOTER),
			new SequentialCommandGroup(
				enableChangeStateAutomatically(false),
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
				).withTimeout(Timeouts.TRANSFER_ARM_SHOOTER_SECONDS),//.until(this::isObjectInFunnel)
				enableChangeStateAutomatically(true),
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
		).handleInterrupt(() -> enableChangeStateAutomatically(true).schedule());
	}

	private Command intakeOuttake() {
		return new ParallelCommandGroup(
			enableChangeStateAutomatically(true),
			setCurrentStateName(RobotState.INTAKE_OUTTAKE),
			rollerStateHandler.setState(RollerState.ROLL_OUT),
			intakeStateHandler.setState(IntakeState.OUTTAKE),
			funnelStateHandler.setState(FunnelState.OUTTAKE),
			pivotStateHandler.setState(PivotState.INTAKE),
			elbowStateHandler.setState(ElbowState.MANUAL),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			wristStateHandler.setState(WristState.IN_ARM),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE)
		).handleInterrupt(() -> enableChangeStateAutomatically(true).schedule());
	}

	private Command armOuttake() {
		return new ParallelCommandGroup(
			enableChangeStateAutomatically(true),
			setCurrentStateName(RobotState.ARM_OUTTAKE),
			rollerStateHandler.setState(RollerState.FAST_ROLL_IN),
			elbowStateHandler.setState(ElbowState.ARM_INTAKE),
			wristStateHandler.setState(WristState.DEFAULT),
			intakeStateHandler.setState(IntakeState.STOP),
			funnelStateHandler.setState(FunnelState.SLOW_OUTTAKE),
			pivotStateHandler.setState(PivotState.INTAKE),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE)
		).handleInterrupt(() -> enableChangeStateAutomatically(true).schedule());
	}

	private Command intakeWithFlywheel() {
	return new ParallelCommandGroup(
		setCurrentStateName(RobotState.INTAKE_WITH_FLYWHEEL),
		new SequentialCommandGroup(
			enableChangeStateAutomatically(false),
			new ParallelCommandGroup(
				intakeStateHandler.setState(IntakeState.INTAKE),
				rollerStateHandler.setState(RollerState.ROLL_IN),
				funnelStateHandler.setState(FunnelState.STOP)
			).until(this::isObjectInIntake),
			new ParallelCommandGroup(
				noteInRumble(),
				intakeStateHandler.setState(IntakeState.INTAKE_WITH_FUNNEL),
				funnelStateHandler.setState(FunnelState.INTAKE),
				rollerStateHandler.setState(RollerState.ROLL_IN)
			).withTimeout(Timeouts.INTAKE_FUNNEL_SECONDS),//.until(this::isObjectInFunnel)
			enableChangeStateAutomatically(true),
			new ParallelCommandGroup(
				intakeStateHandler.setState(IntakeState.STOP),
				rollerStateHandler.setState(RollerState.STOP),
				funnelStateHandler.setState(FunnelState.STOP)
			)
		),
		flywheelStateHandler.setState(FlywheelState.PRE_SPEAKER),
		pivotStateHandler.setState(PivotState.IDLE),
		elbowStateHandler.setState(ElbowState.INTAKE),
		wristStateHandler.setState(WristState.IN_ARM),
		swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.NOTE))
		).handleInterrupt(() -> enableChangeStateAutomatically(true).schedule());
	}
	//@formatter:on

}
