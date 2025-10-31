package frc.robot.statemachine.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeState;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.elevator.ElevatorState;
import org.littletonrobotics.junction.Logger;


public class TargetChecks {

	private final Superstructure superstructure;


	public static final double ELEVATOR_HEIGHT_TOLERANCE_METERS = 0.05;

	public static final Rotation2d ARM_POSITION_TOLERANCE = Rotation2d.fromDegrees(1.5);
	public static final Rotation2d ARM_INTERPOLATION_POSITION_TOLERANCE = Rotation2d.fromDegrees(0.5);
	public static final Rotation2d ARM_ALGAE_TRANSFER_POSITION_TOLERANCE = Rotation2d.fromDegrees(5);
	public static final Rotation2d ARM_ALGAE_RELEASE_POSITION_TOLERANCE = Rotation2d.fromDegrees(10);

	public static final Rotation2d PIVOT_POSITION_TOLERANCE = Rotation2d.fromDegrees(3);


	public TargetChecks(Superstructure superstructure) {
		this.superstructure = superstructure;
	}


	public boolean isPreNetReady() {
		return superstructure.getElevatorStateHandler().isAtState(ElevatorState.NET)
			&& superstructure.getArmStateHandler().isAtState(ArmState.PRE_NET);
	}

	public boolean isPreScoreReady() {
		ScoreLevel targetScoreLevel = ScoringHelpers.targetScoreLevel;
		ArmState targetArmState = targetScoreLevel == ScoreLevel.L4 ? targetScoreLevel.getArmScore() : targetScoreLevel.getArmPreScore();
		return superstructure.getElevatorStateHandler().isAtState(targetScoreLevel.getElevatorPreScore())
			&& superstructure.getArmStateHandler().isAtState(targetArmState);
	}

	public boolean isReadyToScore() {
		ScoreLevel targetScoreLevel = ScoringHelpers.targetScoreLevel;
		return superstructure.getElevatorStateHandler().isAtState(targetScoreLevel.getElevatorScore())
			&& superstructure.getArmStateHandler().isAtState(targetScoreLevel.getArmScore());
	}

	public boolean isReadyToOuttakeAlgae() {
		return superstructure.getElevatorStateHandler().isAtState(ElevatorState.ALGAE_OUTTAKE)
			&& superstructure.getArmStateHandler().isAtState(ArmState.ALGAE_OUTTAKE, ARM_ALGAE_RELEASE_POSITION_TOLERANCE);
	}

	public boolean isReadyToProcessor() {
		return superstructure.getElevatorStateHandler().isAtState(ElevatorState.PROCESSOR_OUTTAKE)
			&& superstructure.getArmStateHandler().isAtState(ArmState.PROCESSOR_OUTTAKE, ARM_ALGAE_RELEASE_POSITION_TOLERANCE);
	}

	public boolean isReadyToTransferAlgae() {
		boolean arm = superstructure.getArmStateHandler().isAtState(ArmState.TRANSFER_ALGAE_FROM_INTAKE, ARM_ALGAE_TRANSFER_POSITION_TOLERANCE);
		boolean elevator = superstructure.getElevatorStateHandler().isAtState(ElevatorState.TRANSFER_ALGAE_FROM_INTAKE);
		boolean algaeIntake = superstructure.getAlgaeIntakeStateHandler().getCurrentState() == AlgaeIntakeState.HOLD_ALGAE;

		Logger.recordOutput("isReadyToTransferAlgae/armInStateAndPos", arm);
		Logger.recordOutput("isReadyToTransferAlgae/elevatorInStateAndPos", elevator);
		Logger.recordOutput("isReadyToTransferAlgae/algaeIntakeInState", algaeIntake);

		return arm && elevator && algaeIntake;
	}

}
