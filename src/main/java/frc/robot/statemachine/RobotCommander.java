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
		this.currentState = RobotState.DRIVE;

		setDefaultCommand(new DeferredCommand(() -> endState(currentState), Set.of(this)));
	}

	private boolean isReadyToScore(ScoreLevel level, Branch branch) {
		ChassisSpeeds allianceRelativeSpeeds = swerve.getAllianceRelativeVelocity();
		Translation2d rotated = new Translation2d(allianceRelativeSpeeds.vxMetersPerSecond, allianceRelativeSpeeds.vyMetersPerSecond)
			.rotateBy(Field.getReefSideMiddle(branch.getReefSide()).getRotation());
		ChassisSpeeds reefRelativeSpeeds = new ChassisSpeeds(rotated.getX(), rotated.getY(), allianceRelativeSpeeds.omegaRadiansPerSecond);
		Pose2d reefRelativePose = robot.getPoseEstimator()
			.getEstimatedPose()
			.rotateBy(Field.getReefSideMiddle(branch.getReefSide()).getRotation());
		Pose2d reefRelativeScoringPose = ScoringHelpers
			.getRobotScoringPose(branch, StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS)
			.rotateBy(Field.getReefSideMiddle(branch.getReefSide()).getRotation());

		return superstructure.isReadyToScore(level) && switch (level) {
			case L1 ->
				isAtPose(
					reefRelativePose,
					reefRelativeScoringPose,
					reefRelativeSpeeds,
					Tolerances.REEF_RELATIVE_L1_SCORING_POSITION,
					Tolerances.REEF_RELATIVE_L1_SCORING_DEADBANDS
				);
			case L2, L3, L4 ->
				isAtPose(
					reefRelativePose,
					reefRelativeScoringPose,
					reefRelativeSpeeds,
					Tolerances.REEF_RELATIVE_SCORING_POSITION,
					Tolerances.REEF_RELATIVE_SCORING_DEADBANDS
				);
		};
	}

	public boolean isAtPose(Pose2d currentPose, Pose2d targetPose, ChassisSpeeds currentSpeeds, Pose2d tolerances, Pose2d deadbands) {
		boolean isAtX = MathUtil.isNear(targetPose.getX(), currentPose.getX(), tolerances.getX());
		boolean isAtY = MathUtil.isNear(targetPose.getY(), currentPose.getY(), tolerances.getY());
		boolean isAtHeading = ToleranceMath.isNearWrapped(targetPose.getRotation(), currentPose.getRotation(), tolerances.getRotation());
		boolean isStill = SwerveMath.isStill(currentSpeeds, deadbands);
		return isAtX && isAtY && isAtHeading && isStill;
	}

	public Command setState(RobotState state) {
		return switch (state) {
			case DRIVE -> drive();
			case INTAKE -> intake();
			case OUTTAKE -> outtake();
			case ALIGN_REEF -> alignReef();
			case PRE_L1 -> preScore(ScoreLevel.L1);
			case PRE_L2 -> preScore(ScoreLevel.L2);
			case PRE_L3 -> preScore(ScoreLevel.L3);
			case PRE_L4 -> preScore(ScoreLevel.L4);
			case L1 -> score(ScoreLevel.L1);
			case L2 -> score(ScoreLevel.L2);
			case L3 -> score(ScoreLevel.L3);
			case L4 -> score(ScoreLevel.L4);
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

	private Command preScore(ScoreLevel scoreLevel) {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.preScore(scoreLevel),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.REEF))
			),
			switch (scoreLevel) {
				case L1 -> RobotState.PRE_L1;
				case L2 -> RobotState.PRE_L2;
				case L3 -> RobotState.PRE_L3;
				case L4 -> RobotState.PRE_L4;
			}
		);
	}

	private Command score(ScoreLevel scoreLevel) {
		return asSubsystemCommand(
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH)),
					superstructure.preScore(scoreLevel)
				).until(() -> isReadyToScore(scoreLevel, ScoringHelpers.targetBranch)),
				superstructure.score(scoreLevel)
			),
			switch (scoreLevel) {
				case L1 -> RobotState.L1;
				case L2 -> RobotState.L2;
				case L3 -> RobotState.L3;
				case L4 -> RobotState.L4;
			}
		);
	}

	private Command asSubsystemCommand(Command command, RobotState state) {
		return new ParallelCommandGroup(asSubsystemCommand(command, state.name()), new InstantCommand(() -> currentState = state));
	}

	private Command endState(RobotState state) {
		return switch (state) {
			case INTAKE, OUTTAKE, DRIVE, ALIGN_REEF -> drive();
			case PRE_L1, L1 -> preScore(ScoreLevel.L1);
			case PRE_L2, L2 -> preScore(ScoreLevel.L2);
			case PRE_L3, L3 -> preScore(ScoreLevel.L3);
			case PRE_L4, L4 -> preScore(ScoreLevel.L4);
		};
	}

}
