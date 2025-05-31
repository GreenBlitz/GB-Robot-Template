package frc.robot.subsystems.algaeIntake;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.signal.AngleSignal;
import frc.robot.subsystems.GBSubsystem;

public class AlgaeIntake extends GBSubsystem {

	private final ControllableMotor pivot;
	private final ControllableMotor rollers;

	private final IRequest<Rotation2d> pivotPositionRequest;
	private final IRequest<Rotation2d> rollersVelocityRequest;
	private final IRequest<Double> rollersVoltageRequest;

	private final InputSignal<Rotation2d> pivotPositionSignal;
    private final InputSignal<Double> pivotVoltageSignal;
    private final InputSignal<Rotation2d> rollersVelocitySignal;
    private final InputSignal<Double> rollersVoltageSignal;

    private final AlgaeIntakeCommandsBuilder commandsBuilder;

	public AlgaeIntake(
		String logPath,
		ControllableMotor pivot,
		ControllableMotor rollers,
		IRequest<Rotation2d> pivotPositionRequest,
		IRequest<Rotation2d> rollersVelocityRequest,
		IRequest<Double> rollersVoltageRequest,
        InputSignal<Rotation2d> pivotPositionSignal,
        InputSignal<Double> pivotVoltageSignal,
        InputSignal<Rotation2d> rollersVelocitySignal,
        InputSignal<Double> rollersVoltageSignal
	) {
		super(logPath);

		this.pivot = pivot;
		this.rollers = rollers;

		this.pivotPositionRequest = pivotPositionRequest;
		this.rollersVelocityRequest = rollersVelocityRequest;
		this.rollersVoltageRequest = rollersVoltageRequest;

        this.pivotPositionSignal = pivotPositionSignal;
        this.pivotVoltageSignal = pivotVoltageSignal;
        this.rollersVelocitySignal = rollersVelocitySignal;
        this.rollersVoltageSignal = rollersVoltageSignal;

        periodic();

        this.commandsBuilder = new AlgaeIntakeCommandsBuilder(this);
	}

    public AlgaeIntakeCommandsBuilder getCommandsBuilder() {
        return commandsBuilder;
    }

    public Rotation2d getPivotPosition(){
        return pivotPositionSignal.getLatestValue();
    }

    public double getPivotVoltage(){
        return pivotVoltageSignal.getLatestValue();
    }

    public Rotation2d getRollersVelocityRotation2dPerSecond(){
        return rollersVelocitySignal.getLatestValue();
    }

    public double getRollersVoltage(){
        return rollersVoltageSignal.getLatestValue();
    }

    @Override
    protected void subsystemPeriodic() {
        updateInputs();
    }

    private void updateInputs(){
        pivot.updateInputs(pivotPositionSignal, pivotVoltageSignal);
        rollers.updateInputs(rollersVelocitySignal, rollersVoltageSignal);

        pivot.updateSimulation();
        rollers.updateSimulation();
    }

    public void setBrake(boolean brake){
        pivot.setBrake(brake);
        rollers.setBrake(brake);
    }

    protected void setTargetPivotPosition(Rotation2d targetPosition) {
        pivot.applyRequest(pivotPositionRequest.withSetPoint(targetPosition));
    }

    protected void setTargetRollersVelocity(Rotation2d targetVelocityMPS){
        rollers.applyRequest(rollersVelocityRequest.withSetPoint(targetVelocityMPS));
    }

    protected void setRollersVoltage(double voltage){
        rollers.applyRequest(rollersVoltageRequest.withSetPoint(voltage));
    }
}
