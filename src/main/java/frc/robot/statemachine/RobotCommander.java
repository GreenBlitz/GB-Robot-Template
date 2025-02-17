package frc.robot.statemachine;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.*;
import frc.constants.field.Field;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.superstructure.Superstructure;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;
import frc.utils.pose.PoseUtil;
import org.json.simple.parser.ParseException;

import java.io.IOException;
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
		this.superstructure = new Superstructure("StateMachine/Superstructure", robot);
		this.currentState = RobotState.DRIVE;
	}
	
	public Superstructure getSuperstructure() {
		return superstructure;
	}
	
	/**
	 * Check if robot at pose but relative to target branch. Y-axis is vertical to the branch. X-axis is horizontal to the branch So when you
	 * check if robot in place in y-axis its in parallel to the reef side.
	 */
	private boolean isAtReefScoringPose(
			double scoringPoseDistanceFromReefMeters,
			Pose2d l1Tolerances,
			Pose2d l1Deadbands,
			Pose2d tolerances,
			Pose2d deadbands
	) {
		Rotation2d reefAngle = Field.getReefSideMiddle(ScoringHelpers.getTargetBranch().getReefSide()).getRotation();
		
		Pose2d reefRelativeTargetPose = ScoringHelpers
				.getRobotBranchScoringPose(ScoringHelpers.getTargetBranch(), scoringPoseDistanceFromReefMeters)
				.rotateBy(reefAngle.unaryMinus());
		Pose2d reefRelativeRobotPose = robot.getPoseEstimator().getEstimatedPose().rotateBy(reefAngle.unaryMinus());
		
		ChassisSpeeds allianceRelativeSpeeds = swerve.getAllianceRelativeVelocity();
		ChassisSpeeds reefRelativeSpeeds = SwerveMath
				.robotToAllianceRelativeSpeeds(allianceRelativeSpeeds, Field.getAllianceRelative(reefAngle.unaryMinus()));
		
		return switch (ScoringHelpers.targetScoreLevel) {
			case L1 ->
					PoseUtil.isAtPose(reefRelativeRobotPose, reefRelativeTargetPose, reefRelativeSpeeds, l1Tolerances, l1Deadbands);
			case L2, L3, L4 ->
					PoseUtil.isAtPose(reefRelativeRobotPose, reefRelativeTargetPose, reefRelativeSpeeds, tolerances, deadbands);
		};
	}
	
	private boolean isAtReefScoringPose(double scoringPoseDistanceFromReefMeters, Translation2d tolerances) {
		Rotation2d reefAngle = Field.getReefSideMiddle(ScoringHelpers.getTargetBranch().getReefSide()).getRotation();
		
		Translation2d reefRelativeTargetPose = ScoringHelpers
				.getRobotBranchScoringPose(ScoringHelpers.getTargetBranch(), scoringPoseDistanceFromReefMeters)
				.rotateBy(reefAngle.unaryMinus())
				.getTranslation();
		Translation2d reefRelativeRobotPose = robot.getPoseEstimator().getEstimatedPose().rotateBy(reefAngle.unaryMinus()).getTranslation();
		
		return PoseUtil.isAtTranslation(reefRelativeRobotPose, reefRelativeTargetPose, tolerances);
	}
	
	private boolean isReadyToOpenSuperstructure() {
		return isAtReefScoringPose(
				StateMachineConstants.OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS,
				Tolerances.REEF_RELATIVE_L1_OPEN_SUPERSTRUCTURE_POSITION,
				Tolerances.REEF_RELATIVE_L1_OPEN_SUPERSTRUCTURE_DEADBANDS,
				Tolerances.REEF_RELATIVE_OPEN_SUPERSTRUCTURE_POSITION,
				Tolerances.REEF_RELATIVE_OPEN_SUPERSTRUCTURE_DEADBANDS
		);
	}
	
	private boolean isPreScoreReady() {
		return superstructure.isPreScoreReady()
				&& isAtReefScoringPose(
				StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS,
				Tolerances.REEF_RELATIVE_L1_SCORING_POSITION,
				Tolerances.REEF_RELATIVE_L1_SCORING_DEADBANDS,
				Tolerances.REEF_RELATIVE_SCORING_POSITION,
				Tolerances.REEF_RELATIVE_SCORING_DEADBANDS
		);
	}
	
	/**
	 * Checks if the robot is out of the safe zone to close the superstructure
	 */
	public boolean isReadyToCloseSuperstructure() {
		return !isAtReefScoringPose(
				StateMachineConstants.CLOSE_SUPERSTRUCTURE_DISTANCE_FROM_BRANCH_METERS,
				StateMachineConstants.CLOSE_SUPERSTRUCTURE_LENGTH_AND_WIDTH
		);
	}
	
	public boolean isReadyToScore() {
		return superstructure.isReadyToScore()
				&& isAtReefScoringPose(
				StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS,
				Tolerances.REEF_RELATIVE_L1_SCORING_POSITION,
				Tolerances.REEF_RELATIVE_L1_SCORING_DEADBANDS,
				Tolerances.REEF_RELATIVE_SCORING_POSITION,
				Tolerances.REEF_RELATIVE_SCORING_DEADBANDS
		);
	}
	
	public Command setState(RobotState state) {
		return switch (state) {
			case DRIVE -> drive();
			case INTAKE -> intake();
			case OUTTAKE -> outtake();
			case ALIGN_REEF -> alignReef();
			case ARM_PRE_SCORE -> armPreScore();
			case PRE_SCORE -> preScore();
			case SCORE_WITHOUT_RELEASE -> scoreWithoutRelease();
			case SCORE -> score();
		};
	}
	
	public Command fullyScore() {
		return new SequentialCommandGroup(
				armPreScore().until(this::isReadyToOpenSuperstructure),
				preScore().until(this::isPreScoreReady),
				scoreWithoutRelease().until(this::isReadyToScore),
				score()
		);
	}
	
	public Command scoreForButton() {
		return new SequentialCommandGroup(scoreWithoutRelease().until(this::isReadyToScore), score());
	}
	
	public Command fullyPreScore() {
		return new SequentialCommandGroup(
				armPreScore().until(this::isReadyToOpenSuperstructure),
				preScore().until(this::isPreScoreReady),
				scoreWithoutRelease()
		);
	}
	
	private Command drive() {
		return asSubsystemCommand(
				new ParallelCommandGroup(superstructure.idle(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
				RobotState.DRIVE
		);
	}
	
	private Command intake() {
		return asSubsystemCommand(
				new ParallelDeadlineGroup(
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
	
	private Command armPreScore() {
		return asSubsystemCommand(
				new ParallelCommandGroup(
						superstructure.armPreScore(),
						swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
				),
				RobotState.ARM_PRE_SCORE
		);
	}
	
	private Command preScore() {
		return asSubsystemCommand(
				new ParallelCommandGroup(
						superstructure.preScore(),
						swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
				),
				RobotState.PRE_SCORE
		);
	}
	
	private Command scoreWithoutRelease() {
		return asSubsystemCommand(
				new ParallelCommandGroup(
						superstructure.scoreWithoutRelease(),
						swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
				),
				RobotState.SCORE_WITHOUT_RELEASE
		);
	}
	
	private Command score() {
		return asSubsystemCommand(
				new ParallelDeadlineGroup(
						superstructure.scoreWithRelease(),
						swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
				),
				RobotState.SCORE
		);
	}
	
	private Command closeAfterScore() {
		return new SequentialCommandGroup(
				new ParallelCommandGroup(superstructure.preScore(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE))
						.until(this::isReadyToCloseSuperstructure),
				drive()
		);
	}
	
	private Command asSubsystemCommand(Command command, RobotState state) {
		return new ParallelCommandGroup(asSubsystemCommand(command, state.name()), new InstantCommand(() -> currentState = state));
	}
	
	private Command endState(RobotState state) {
		return switch (state) {
			case INTAKE, OUTTAKE, DRIVE, ALIGN_REEF -> drive();
			case ARM_PRE_SCORE -> armPreScore();
			case PRE_SCORE -> preScore();
			case SCORE, SCORE_WITHOUT_RELEASE -> closeAfterScore();
		};
	}
	
	public Command driveToReefWaitPose() {
		return asSubsystemCommand(
				new DeferredCommand(
						() -> swerve.getCommandsBuilder()
								.pathToPose(
										() -> robot.getPoseEstimator().getEstimatedPose(),
										() -> ScoringHelpers.getRobotBranchScoringPose(
												ScoringHelpers.getTargetBranch(),
												StateMachineConstants.OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS
										)
								),
						Set.of(this, swerve)
				),
				"driving to waiting pose of: " + ScoringHelpers.getTargetBranch().name() + " " + ScoringHelpers.targetScoreLevel.name()
		);
	}
	
	public Command driveToReefTargetPose() {
		ProfiledPIDController profiledPIDController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(360, 180));
		profiledPIDController.enableContinuousInput(0, 360);
		return asSubsystemCommand(
				new DeferredCommand(
						() -> swerve.getCommandsBuilder()
								.driveToPose(
										() -> robot.getPoseEstimator().getEstimatedPose(),
										() -> ScoringHelpers.getRobotBranchScoringPose(
												ScoringHelpers.getTargetBranch(),
												StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS
										),
										swerve::getChassisSpeeds,
										new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(2, 2)),
										new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(2, 2)),
										profiledPIDController
								),
						Set.of(this, swerve)
				),
				"driving to waiting pose of: " + ScoringHelpers.getTargetBranch().name() + " " + ScoringHelpers.targetScoreLevel.name()
		);
	}
	
	public Command autoScore() {
		return asSubsystemCommand(
				Commands.sequence(
						Commands.parallel(superstructure.armPreScore(), driveToReefWaitPose())
								.until(
										() -> isAtReefScoringPose(
												StateMachineConstants.OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS,
												Tolerances.REEF_RELATIVE_OPEN_SUPERSTRUCTURE_POSITION.getTranslation()
										)
								),
						Commands
								.parallel(
										driveToReefTargetPose(),
										Commands.sequence(superstructure.preScore().until(this::isPreScoreReady), superstructure.scoreWithRelease())
								)
								.until(superstructure::isCoralOut)
				),
				
				"auto score"
		);
	}
	
	private PathPlannerPath getPathByTargetBranch() {
		try {
			return PathPlannerPath.fromPathFile("a");
		} catch (IOException e) {
			throw new RuntimeException(e);
		} catch (ParseException e) {
			throw new RuntimeException(e);
		}
	}
	
	public Command driveToReef() {
		
		return asSubsystemCommand(AutoBuilder.pathfindThenFollowPath(
						getPathByTargetBranch(),
						AutonomousConstants.REAL_TIME_CONSTRAINTS
				),
				"pathfindThenFollowPath"
		);
	}
	
}
