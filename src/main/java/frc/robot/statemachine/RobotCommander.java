package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.robot.Robot;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.robot.statemachine.superstructure.Superstructure;
import frc.robot.statemachine.superstructure.SuperstructureState;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;
import frc.utils.pose.PoseUtil;

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
		this.currentState = RobotState.DRIVE;

		setDefaultCommand(new DeferredCommand(() -> endState(currentState), Set.of(this)));
	}

	/**
	 * Checks if elevator and arm in place and is robot at pose but relative to target branch. Y-axis is vertical to the branch. X-axis is
	 * horizontal to the branch So when you check if robot in place in y-axis its in parallel to the reef side.
	 */
	private boolean isReadyToScore(ScoreLevel level, Branch branch) {
		Rotation2d reefAngle = Field.getReefSideMiddle(branch.getReefSide()).getRotation();

		Pose2d reefRelativeTargetPose = ScoringHelpers.getRobotScoringPose(branch, StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS)
			.rotateBy(reefAngle.unaryMinus());
		Pose2d reefRelativeRobotPose = robot.getPoseEstimator().getEstimatedPose().rotateBy(reefAngle.unaryMinus());

		ChassisSpeeds allianceRelativeSpeeds = swerve.getAllianceRelativeVelocity();
		ChassisSpeeds reefRelativeSpeeds = SwerveMath
			.robotToAllianceRelativeSpeeds(allianceRelativeSpeeds, Field.getAllianceRelative(reefAngle.unaryMinus()));

		return superstructure.isReadyToScore(level) && switch (level) {
			case L1 ->
				PoseUtil.isAtPose(
					reefRelativeRobotPose,
					reefRelativeTargetPose,
					reefRelativeSpeeds,
					Tolerances.REEF_RELATIVE_L1_SCORING_POSITION,
					Tolerances.REEF_RELATIVE_L1_SCORING_DEADBANDS
				);
			case L2, L3, L4 ->
				PoseUtil.isAtPose(
					reefRelativeRobotPose,
					reefRelativeTargetPose,
					reefRelativeSpeeds,
					Tolerances.REEF_RELATIVE_SCORING_POSITION,
					Tolerances.REEF_RELATIVE_SCORING_DEADBANDS
				);
		};
	}

	public Command setState(RobotState state) {
		return switch (state) {
			case DRIVE -> drive();
			case INTAKE -> intake();
			case OUTTAKE -> outtake();
			case ALIGN_REEF -> alignReef();
			case PRE_L1 -> preL1();
			case PRE_L2 -> preL2();
			case PRE_L3 -> preL3();
			case PRE_L4 -> preL4();
			case L1 -> scoreL1();
			case L2 -> scoreL2();
			case L3 -> scoreL3();
			case L4 -> scoreL4();
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

	private Command preScore(RobotState robotState, SuperstructureState superstructureState, ScoreLevel scoreLevel) {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.preScore(scoreLevel, superstructureState),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.REEF))
			),
			robotState
		);
	}

	private Command preL1() {
		return preScore(RobotState.PRE_L1, SuperstructureState.PRE_L1, ScoreLevel.L1);
	}

	private Command preL2() {
		return preScore(RobotState.PRE_L2, SuperstructureState.PRE_L2, ScoreLevel.L2);
	}

	private Command preL3() {
		return preScore(RobotState.PRE_L3, SuperstructureState.PRE_L3, ScoreLevel.L2);
	}

	private Command preL4() {
		return preScore(RobotState.PRE_L4, SuperstructureState.PRE_L4, ScoreLevel.L4);
	}

	private Command score(RobotState robotState, SuperstructureState superstructureState, ScoreLevel scoreLevel) {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH)),
					superstructure.preScore(scoreLevel, superstructureState)
				).until(() -> isReadyToScore(scoreLevel, ScoringHelpers.targetBranch)),
				superstructure.score(scoreLevel, superstructureState)
			),
			robotState
		);
	}

	private Command scoreL1() {
		return score(RobotState.L1, SuperstructureState.SCORE_L1, ScoreLevel.L1);
	}

	private Command scoreL2() {
		return score(RobotState.L2, SuperstructureState.SCORE_L2, ScoreLevel.L2);
	}

	private Command scoreL3() {
		return score(RobotState.L3, SuperstructureState.SCORE_L3, ScoreLevel.L3);
	}

	private Command scoreL4() {
		return score(RobotState.L4, SuperstructureState.SCORE_L4, ScoreLevel.L4);
	}

	private Command asSubsystemCommand(Command command, RobotState state) {
		return new ParallelCommandGroup(asSubsystemCommand(command, state.name()), new InstantCommand(() -> currentState = state));
	}

	private Command endState(RobotState state) {
		return switch (state) {
			case INTAKE, OUTTAKE, DRIVE, ALIGN_REEF -> drive();
			case PRE_L1, L1 -> preL1();
			case PRE_L2, L2 -> preL2();
			case PRE_L3, L3 -> preL3();
			case PRE_L4, L4 -> preL4();
		};
	}

}
