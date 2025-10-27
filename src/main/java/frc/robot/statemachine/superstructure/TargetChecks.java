package frc.robot.statemachine.superstructure;

import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.Tolerances;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeState;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeStateHandler;
import frc.robot.subsystems.arm.ArmState;
import frc.robot.subsystems.arm.ArmStateHandler;
import frc.robot.subsystems.climb.ClimbStateHandler;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.elevator.ElevatorStateHandler;
import frc.robot.subsystems.endeffector.EndEffectorStateHandler;
import org.littletonrobotics.junction.Logger;


public class TargetChecks {

	private final ElevatorStateHandler elevatorStateHandler;
	private final ArmStateHandler armStateHandler;
	private final EndEffectorStateHandler endEffectorStateHandler;
	private final ClimbStateHandler climbStateHandler;
	private final AlgaeIntakeStateHandler algaeIntakeStateHandler;


	public TargetChecks(
		ElevatorStateHandler elevatorStateHandler,
		ArmStateHandler armStateHandler,
		EndEffectorStateHandler endEffectorStateHandler,
		ClimbStateHandler climbStateHandler,
		AlgaeIntakeStateHandler algaeIntakeStateHandler
	) {
		this.elevatorStateHandler = elevatorStateHandler;
		this.armStateHandler = armStateHandler;
		this.endEffectorStateHandler = endEffectorStateHandler;
		this.climbStateHandler = climbStateHandler;
		this.algaeIntakeStateHandler = algaeIntakeStateHandler;
	}


	public boolean isPreNetReady() {
		return elevatorStateHandler.isAtState(ElevatorState.NET) && armStateHandler.isAtState(ArmState.PRE_NET);
	}

	public boolean isPreScoreReady() {
		ScoreLevel targetScoreLevel = ScoringHelpers.targetScoreLevel;
		ArmState targetArmState = targetScoreLevel == ScoreLevel.L4 ? targetScoreLevel.getArmScore() : targetScoreLevel.getArmPreScore();
		return elevatorStateHandler.isAtState(targetScoreLevel.getElevatorPreScore()) && armStateHandler.isAtState(targetArmState);
	}

	public boolean isReadyToScore() {
		ScoreLevel targetScoreLevel = ScoringHelpers.targetScoreLevel;
		return elevatorStateHandler.isAtState(targetScoreLevel.getElevatorScore()) && armStateHandler.isAtState(targetScoreLevel.getArmScore());
	}

	public boolean isReadyToOuttakeAlgae() {
		return elevatorStateHandler.isAtState(ElevatorState.ALGAE_OUTTAKE)
			&& armStateHandler.isAtState(ArmState.ALGAE_OUTTAKE, Tolerances.ALGAE_RELEASE_ARM_POSITION);
	}

	public boolean isReadyToProcessor() {
		return elevatorStateHandler.isAtState(ElevatorState.PROCESSOR_OUTTAKE)
			&& armStateHandler.isAtState(ArmState.PROCESSOR_OUTTAKE, Tolerances.ALGAE_RELEASE_ARM_POSITION);
	}

	public boolean isReadyToTransferAlgae() {
		boolean arm = armStateHandler.isAtState(ArmState.TRANSFER_ALGAE_FROM_INTAKE, Tolerances.ARM_ALGAE_TRANSFER_POSITION);
		boolean elevator = elevatorStateHandler.isAtState(ElevatorState.TRANSFER_ALGAE_FROM_INTAKE);
		boolean algaeIntake = algaeIntakeStateHandler.getCurrentState() == AlgaeIntakeState.HOLD_ALGAE;

		Logger.recordOutput("isReadyToTransferAlgae/armInStateAndPos", arm);
		Logger.recordOutput("isReadyToTransferAlgae/elevatorInStateAndPos", elevator);
		Logger.recordOutput("isReadyToTransferAlgae/algaeIntakeInState", algaeIntake);

		return arm && elevator && algaeIntake;
	}

}
