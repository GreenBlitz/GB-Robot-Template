package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.JoysticksBindings;
import frc.robot.Robot;
import frc.robot.constants.Field;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.elbow.ElbowState;
import frc.robot.subsystems.elbow.ElbowStateHandler;
import frc.robot.subsystems.flywheel.FlywheelState;
import frc.robot.subsystems.flywheel.FlywheelStateHandler;
import frc.robot.subsystems.funnel.FunnelState;
import frc.robot.subsystems.funnel.FunnelStateHandler;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.intake.IntakeStateHandler;
import frc.robot.subsystems.pivot.PivotInterpolationMap;
import frc.robot.subsystems.pivot.PivotState;
import frc.robot.subsystems.pivot.PivotStateHandler;
import frc.robot.subsystems.roller.RollerState;
import frc.robot.subsystems.roller.RollerStateHandler;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import frc.robot.subsystems.wrist.WristState;
import frc.robot.subsystems.wrist.WristStateHandler;
import frc.utils.joysticks.Axis;
import frc.utils.joysticks.SmartJoystick;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class Superstructure extends GBSubsystem {

	private interface RobotCommandGenerator {

		Command makeRobotCommand(Command command, RobotState state);

	}

	private final RobotCommandGenerator robotCommandGenerator = (command, state) -> {
		command.addRequirements(this);
		return command.beforeStarting(setCurrentStateName(state));
	};

	private static final double NOTE_IN_RUMBLE_POWER = 0.5;

	private final Robot robot;
	private final Swerve swerve;
	private final ElbowStateHandler elbowStateHandler;
	private final FlywheelStateHandler flywheelStateHandler;
	private final FunnelStateHandler funnelStateHandler;
	private final IntakeStateHandler intakeStateHandler;
	private final PivotStateHandler pivotStateHandler;
	private final RollerStateHandler rollerStateHandler;
	private final WristStateHandler wristStateHandler;
	private final EndBehaviorManager endBehaviorManager;
	private final SmartJoystick mainJoystick;

	private RobotState currentState;

	public Superstructure(String logPath, Robot robot) {
		super(logPath);
		this.robot = robot;
		this.swerve = robot.getSwerve();
		this.elbowStateHandler = new ElbowStateHandler(robot.getElbow());
		this.flywheelStateHandler = new FlywheelStateHandler(robot.getFlywheel());
		this.funnelStateHandler = new FunnelStateHandler(robot.getFunnel());
		this.intakeStateHandler = new IntakeStateHandler(robot.getIntake());
		this.pivotStateHandler = new PivotStateHandler(robot.getPivot(), Optional.of(() -> robot.getPoseEstimator().getEstimatedPose()));
		this.rollerStateHandler = new RollerStateHandler(robot.getRoller());
		this.wristStateHandler = new WristStateHandler(robot.getWrist());
		this.mainJoystick = JoysticksBindings.getMainJoystick();

		this.endBehaviorManager = new EndBehaviorManager(this);
		setDefaultCommand(endBehaviorManager.endState(currentState).asProxy()); // todo - test
	}

	public RobotState getCurrentState() {
		return currentState;
	}

	public void logStatus() {
		Logger.recordOutput(getLogPath() + "CurrentState", currentState);
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

	private boolean isReadyToShootClose() {
		boolean isPivotReady = robot.getPivot().isAtPosition(PivotState.PRE_SPEAKER.getTargetPosition(), Tolerances.PIVOT_POSITION);

		boolean isFlywheelReady = robot.getFlywheel()
			.isAtVelocities(
				FlywheelState.PRE_SPEAKER.getRightVelocity(),
				FlywheelState.PRE_SPEAKER.getLeftVelocity(),
				Tolerances.FLYWHEEL_VELOCITY_PER_SECOND
			);

		return isFlywheelReady && isPivotReady;
	}

	private boolean isReadyToPass() {
		boolean isPivotReady = robot.getPivot().isAtPosition(PivotState.PASSING.getTargetPosition(), Tolerances.PIVOT_POSITION);

		boolean isFlywheelReady = robot.getFlywheel()
			.isAtVelocities(
				FlywheelState.PASSING.getRightVelocity(),
				FlywheelState.PASSING.getLeftVelocity(),
				Tolerances.FLYWHEEL_VELOCITY_PER_SECOND
			);

		return isFlywheelReady && isPivotReady;
	}

	private boolean isReadyToShootInterpolation() {
		Translation2d robotTranslation2d = robot.getPoseEstimator().getEstimatedPose().getTranslation();

		double metersFromSpeaker = Field.getSpeaker().toTranslation2d().getDistance(robotTranslation2d);
		boolean isPivotReady = robot.getPivot()
			.isAtPosition(Rotation2d.fromRadians(PivotInterpolationMap.METERS_TO_RADIANS.get(metersFromSpeaker)), Tolerances.PIVOT_POSITION);

		boolean isFlywheelReady = robot.getFlywheel()
			.isAtVelocities(
				FlywheelState.PRE_SPEAKER.getRightVelocity(),
				FlywheelState.PRE_SPEAKER.getLeftVelocity(),
				Tolerances.FLYWHEEL_VELOCITY_PER_SECOND
			);

		Rotation2d angleToSpeaker = SwerveMath.getRelativeTranslation(robotTranslation2d, Field.getSpeaker().toTranslation2d()).getAngle();
		boolean isSwerveReady = swerve.isAtHeading(angleToSpeaker);

		return isFlywheelReady && isPivotReady && isSwerveReady;
	}

	private Command setCurrentStateName(RobotState state) {
		return new InstantCommand(() -> currentState = state);
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

	private Command driveByMainJoystick(SwerveState state) {
		return swerve.getCommandsBuilder()
			.driveState(
				() -> mainJoystick.getAxisValue(Axis.LEFT_Y),
				() -> mainJoystick.getAxisValue(Axis.LEFT_X),
				() -> mainJoystick.getAxisValue(Axis.RIGHT_X),
				state
			);
	}

	public Command setState(RobotState state) {
		return robotCommandGenerator.makeRobotCommand(switch (state) {
			case IDLE -> idle();
			case INTAKE -> intake();
			case INTAKE_WITH_FLYWHEEL -> intakeWithFlywheel();
			case ARM_INTAKE -> armIntake();
			case PRE_SPEAKER -> preSpeaker();
			case SPEAKER -> speaker();
			case PRE_AMP -> preAMP();
			case AMP -> amp();
			case ARM_UP -> armUp();
			case TRANSFER_SHOOTER_TO_ARM -> transferShooterToArm();
			case TRANSFER_ARM_TO_SHOOTER -> transferArmToShooter();
			case INTAKE_OUTTAKE -> intakeOuttake();
			case ARM_OUTTAKE -> armOuttake();
			case PASSING -> passing();
		}, state);
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
			wristStateHandler.setState(WristState.DEFAULT),
			driveByMainJoystick(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command intake() {
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(
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
				).until(this::isObjectInFunnel)
			),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			pivotStateHandler.setState(PivotState.IDLE),
			elbowStateHandler.setState(ElbowState.INTAKE),
			wristStateHandler.setState(WristState.IN_ARM),
			driveByMainJoystick(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.NOTE))
		);
	}

	private Command intakeWithFlywheel() {
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(
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
				).until(this::isObjectInFunnel)
			),
			flywheelStateHandler.setState(FlywheelState.PRE_SPEAKER),
			pivotStateHandler.setState(PivotState.IDLE),
			elbowStateHandler.setState(ElbowState.INTAKE),
			wristStateHandler.setState(WristState.IN_ARM),
			driveByMainJoystick(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.NOTE))
		);
	}

	private Command armIntake() {
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(
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
				 .until(() -> robot.getWrist().isAtPosition(WristState.DEFAULT.getPosition(), Tolerances.WRIST_POSITION))
			),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			pivotStateHandler.setState(PivotState.ARM_INTAKE),
			elbowStateHandler.setState(ElbowState.ARM_INTAKE),
			driveByMainJoystick(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.NOTE))
		);
	}

	private Command preSpeaker() {
		return new ParallelCommandGroup(
			rollerStateHandler.setState(RollerState.STOP),
			intakeStateHandler.setState(IntakeState.STOP),
			funnelStateHandler.setState(FunnelState.MANUAL),
			pivotStateHandler.setState(PivotState.INTERPOLATE),
			flywheelStateHandler.setState(FlywheelState.PRE_SPEAKER),
			elbowStateHandler.setState(ElbowState.IDLE),
			wristStateHandler.setState(WristState.IN_ARM),
			driveByMainJoystick(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER))
		);
	}

	private Command speaker() {
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP),
					intakeStateHandler.setState(IntakeState.STOP)
				).until(this::isReadyToShootInterpolation),
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.SHOOT),
					intakeStateHandler.setState(IntakeState.INTAKE_WITH_FUNNEL)
				).until(() -> !isObjectInFunnel())
			),
			rollerStateHandler.setState(RollerState.STOP),
			pivotStateHandler.setState(PivotState.INTERPOLATE),
			flywheelStateHandler.setState(FlywheelState.PRE_SPEAKER),
			elbowStateHandler.setState(ElbowState.INTAKE),
			wristStateHandler.setState(WristState.IN_ARM),
			driveByMainJoystick(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER))
		);
	}

	private Command preAMP() {
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
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
			driveByMainJoystick(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP)),
			rollerStateHandler.setState(RollerState.STOP),
			pivotStateHandler.setState(PivotState.IDLE),
			wristStateHandler.setState(WristState.IN_ARM),
			flywheelStateHandler.setState(FlywheelState.DEFAULT)
		);
	}

	private Command amp() {
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP),
					rollerStateHandler.setState(RollerState.STOP)
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
				).withTimeout(Timeouts.AMP_RELEASE_SECONDS)//.until(() -> !isObjectInRoller())
			),
			driveByMainJoystick(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP)),
			pivotStateHandler.setState(PivotState.IDLE),
			wristStateHandler.setState(WristState.IN_ARM),
			flywheelStateHandler.setState(FlywheelState.DEFAULT)
		);
	}

	private Command armUp() {
		return new ParallelCommandGroup(
			rollerStateHandler.setState(RollerState.MANUAL),
			intakeStateHandler.setState(IntakeState.STOP),
			funnelStateHandler.setState(FunnelState.MANUAL),
			pivotStateHandler.setState(PivotState.IDLE),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			elbowStateHandler.setState(ElbowState.PRE_AMP),
			wristStateHandler.setState(WristState.DEFAULT),
			driveByMainJoystick(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command transferShooterToArm() {
		return new ParallelDeadlineGroup(
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
				).withTimeout(Timeouts.TRANSFER_SHOOTER_ARM_SECONDS),//.until(this::isObjectInRoller),
				new ParallelDeadlineGroup(
					rollerStateHandler.setState(RollerState.AFTER_INTAKE),
					funnelStateHandler.setState(FunnelState.TRANSFER_TO_ARM)
				).withTimeout(Timeouts.INTAKE_ARM_1_ROTATION_SECONDS)
			),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			intakeStateHandler.setState(IntakeState.STOP),
			wristStateHandler.setState(WristState.IN_ARM),
			driveByMainJoystick(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command transferArmToShooter() {
		return new ParallelDeadlineGroup(
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
				).until(this::isObjectInFunnel)
			),
			wristStateHandler.setState(WristState.IN_ARM),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			driveByMainJoystick(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command intakeOuttake() {
		return new ParallelCommandGroup(
			rollerStateHandler.setState(RollerState.ROLL_OUT),
			intakeStateHandler.setState(IntakeState.OUTTAKE),
			funnelStateHandler.setState(FunnelState.OUTTAKE),
			pivotStateHandler.setState(PivotState.INTAKE),
			elbowStateHandler.setState(ElbowState.MANUAL),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			wristStateHandler.setState(WristState.IN_ARM),
			driveByMainJoystick(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command armOuttake() {
		return new ParallelCommandGroup(
			rollerStateHandler.setState(RollerState.FAST_ROLL_IN),
			elbowStateHandler.setState(ElbowState.ARM_INTAKE),
			wristStateHandler.setState(WristState.DEFAULT),
			intakeStateHandler.setState(IntakeState.STOP),
			funnelStateHandler.setState(FunnelState.SLOW_OUTTAKE),
			pivotStateHandler.setState(PivotState.INTAKE),
			flywheelStateHandler.setState(FlywheelState.DEFAULT),
			driveByMainJoystick(SwerveState.DEFAULT_DRIVE)
		);
	}

	private Command passing() {
		return new ParallelDeadlineGroup(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.STOP),
					intakeStateHandler.setState(IntakeState.STOP)
				).until(this::isReadyToPass),
				new ParallelCommandGroup(
					funnelStateHandler.setState(FunnelState.SHOOT),
					intakeStateHandler.setState(IntakeState.INTAKE_WITH_FUNNEL)
				).until(() -> !isObjectInFunnel())
			),
			rollerStateHandler.setState(RollerState.STOP),
			pivotStateHandler.setState(PivotState.PASSING),
			flywheelStateHandler.setState(FlywheelState.PASSING),
			elbowStateHandler.setState(ElbowState.INTAKE),
			wristStateHandler.setState(WristState.IN_ARM),
			driveByMainJoystick(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.PASS))
		);
	}
	//@formatter:on

}
