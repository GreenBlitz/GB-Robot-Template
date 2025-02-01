package frc.robot.statemachine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.robot.Robot;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.robot.statemachine.superstructure.Superstructure;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;
import frc.utils.math.ToleranceMath;

import java.util.Set;

public class RobotCommander extends GBSubsystem {

	private final Robot robot;
	private final Swerve swerve;
	private final Superstructure superstructure;

	private RobotState currentState;

	public RobotCommander(String logPath, Robot robot) {
		super(logPath);
		this.robot = robot;
		this.swerve = robot.getSwerve();
		this.superstructure = robot.getSuperstructure();

		setDefaultCommand(new DeferredCommand(() -> endState(currentState), Set.of(this)));
	}

	private boolean isReadyToScore(ScoreLevel level, Branch branch) {
		ChassisSpeeds allianceRelativeSpeeds = swerve.getAllianceRelativeVelocity();
		Translation2d rotated = new Translation2d(allianceRelativeSpeeds.vxMetersPerSecond, allianceRelativeSpeeds.vyMetersPerSecond)
			.rotateBy(Field.getReefSideMiddle(branch.getReefSide()).getRotation());
		ChassisSpeeds reefRelativeSpeeds = new ChassisSpeeds(rotated.getX(), rotated.getY(), allianceRelativeSpeeds.omegaRadiansPerSecond);
		return superstructure.isReadyToScore(level) && switch (level) {
			case L1 ->
				isAtPose(
					robot.getPoseEstimator().getEstimatedPose().rotateBy(Field.getReefSideMiddle(branch.getReefSide()).getRotation()),
					ScoringHelpers.getRobotScoringPose(branch, StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS)
						.rotateBy(Field.getReefSideMiddle(branch.getReefSide()).getRotation()),
					reefRelativeSpeeds,
					Tolerances.REEF_RELATIVE_L1_SCORING_POSITION.rotateBy(Field.getReefSideMiddle(branch.getReefSide()).getRotation()),
					Tolerances.REEF_RELATIVE_L1_SCORING_DEADBANDS.rotateBy(Field.getReefSideMiddle(branch.getReefSide()).getRotation())
				);
			case L2, L3, L4 ->
				isAtPose(
					robot.getPoseEstimator().getEstimatedPose().rotateBy(Field.getReefSideMiddle(branch.getReefSide()).getRotation()),
					ScoringHelpers.getRobotScoringPose(branch, StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS)
						.rotateBy(Field.getReefSideMiddle(branch.getReefSide()).getRotation()),
					reefRelativeSpeeds,
					Tolerances.REEF_RELATIVE_SCORING_POSITION.rotateBy(Field.getReefSideMiddle(branch.getReefSide()).getRotation()),
					Tolerances.REEF_RELATIVE_SCORING_DEADBANDS.rotateBy(Field.getReefSideMiddle(branch.getReefSide()).getRotation())
				);
		};
	}

	public boolean isAtPose(Pose2d currentPose, Pose2d targetPose, ChassisSpeeds currentSpeeds, Pose2d tolerances, Pose2d deadbands) {
		boolean isAtX = MathUtil.isNear(targetPose.getX(), currentPose.getX(), tolerances.getX());
		boolean isAtY = MathUtil.isNear(targetPose.getY(), currentPose.getY(), tolerances.getY());
		boolean isAtHeading = ToleranceMath.isNearWrapped(targetPose.getRotation(), currentPose.getRotation(), tolerances.getRotation());
		boolean isStopping = SwerveMath.isStill(currentSpeeds, deadbands);
		return isAtX && isAtY && isAtHeading && isStopping;
	}

	public Command setState(RobotState state) {
		return switch (state) {
			case DRIVE -> drive();
			case INTAKE -> intake();
			case L1 -> l1();
			case L2 -> l2();
			case L3 -> l3();
			case L4 -> l4();
			case PRE_L1 -> preL1();
			case PRE_L2 -> preL2();
			case PRE_L3 -> preL3();
			case PRE_L4 -> preL4();
			case OUTTAKE -> outtake();
			case ALIGN_REEF -> alignReef();
		};
	}

	private Command drive() {
		return asSubsystemCommand(
			new ParallelCommandGroup(superstructure.idle(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.DRIVE
		);
	}

	private Command intake() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.intake(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.CORAL_STATION))
			),
			RobotState.INTAKE
		);
	}

	private Command l1() {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH)),
					superstructure.preL1()
				).until(() -> isReadyToScore(ScoreLevel.L1, ScoringHelpers.targetBranch)),
				superstructure.scoreL1()
			),
			RobotState.L1
		);
	}

	private Command l2() {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH)),
					superstructure.preL2()
				).until(() -> isReadyToScore(ScoreLevel.L2, ScoringHelpers.targetBranch)),
				superstructure.scoreL2()
			),
			RobotState.L2
		);
	}

	private Command l3() {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH)),
					superstructure.preL3()
				).until(() -> isReadyToScore(ScoreLevel.L3, ScoringHelpers.targetBranch)),
				superstructure.scoreL3()
			),
			RobotState.L3
		);
	}

	private Command l4() {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH)),
					superstructure.preL4()
				).until(() -> isReadyToScore(ScoreLevel.L4, ScoringHelpers.targetBranch)),
				superstructure.scoreL4()
			),
			RobotState.L4
		);
	}

	private Command preL1() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.preL1(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.REEF))
			),
			RobotState.PRE_L1
		);
	}

	private Command preL2() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.preL2(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
			),
			RobotState.PRE_L2
		);
	}

	private Command preL3() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.preL3(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
			),
			RobotState.PRE_L3
		);
	}

	private Command preL4() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.preL4(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
			),
			RobotState.PRE_L4
		);
	}

	private Command outtake() {
		return asSubsystemCommand(
			new ParallelCommandGroup(superstructure.outtake(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.OUTTAKE
		);
	}

	private Command alignReef() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.idle(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.REEF))
			),
			RobotState.ALIGN_REEF
		);
	}

	private Command asSubsystemCommand(Command command, RobotState state) {
		return new ParallelCommandGroup(asSubsystemCommand(command, state.name()), new InstantCommand(() -> currentState = state));
	}

	private Command endState(RobotState state) {
		return switch (state) {
			case INTAKE, OUTTAKE, DRIVE -> drive();
			case PRE_L1, L1 -> preL1();
			case PRE_L2, L2 -> preL2();
			case PRE_L3, L3 -> preL3();
			case PRE_L4, L4 -> preL4();
			case ALIGN_REEF -> alignReef();
		};
	}

}
