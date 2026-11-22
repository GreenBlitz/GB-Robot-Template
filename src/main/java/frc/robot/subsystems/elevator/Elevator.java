package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.interfaces.VelocityPositionRequest;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Elevator extends GBSubsystem {

	protected final ControllableMotor motor;
	private final InputSignal<Double> voltageSignal;
	private final InputSignal<Double> currentSignal;
	private final InputSignal<Rotation2d> positionSignal;
	private final InputSignal<Rotation2d> velocitySignal;
	private final VelocityPositionRequest request;
	private final ElevatorPhysicalProperties physicalProperties;
    private final ElevatorCommandsBuilder commandsBuilder;
	private double targetPosition;

	public Elevator(
		String logPath,
		ControllableMotor motor,
		InputSignal<Double> voltageSignal,
		InputSignal<Double> currentSignal,
		InputSignal<Rotation2d> positionSignal,
		InputSignal<Rotation2d> velocitySignal,
		VelocityPositionRequest velocityPositionRequest,
		ElevatorPhysicalProperties physicalProperties
	) {
		super(logPath);
		this.motor = motor;
		this.voltageSignal = voltageSignal;
		this.currentSignal = currentSignal;
		this.positionSignal = positionSignal;
		this.velocitySignal = velocitySignal;
		this.request = velocityPositionRequest;
		this.commandsBuilder = new ElevatorCommandsBuilder(this);
		this.targetPosition = 0;
		this.physicalProperties = physicalProperties;
		setDefaultCommand(commandsBuilder.stop());
	}

    public ElevatorCommandsBuilder getCommandsBuilder(){
        return commandsBuilder;
    }

	private Rotation2d getMotorTarget() {
		return physicalProperties.convertFractionalHeightToMotorPosition(targetPosition);
	}

	public double getTargetPositionFractional() {
		return targetPosition;
	}

	public Distance getTargetPositionMeasure() {
		return physicalProperties.convertFractionalHeightToPhysicalHeight(targetPosition);
	}

	public void updateTargetPosition(ElevatorMeasure targetPosition) {
		double targetPositionFraction = physicalProperties.getFractionalHeight(targetPosition);
		if (!(0 <= targetPositionFraction && targetPositionFraction <= 1)) {
			System.out.println("Warning: position not in range, called from:");
			Thread.dumpStack();
		}
		this.targetPosition = targetPositionFraction;
	}

	private boolean isAtPosition(double positionFraction, double tolerance) {
		return MathUtil.isNear(positionFraction, getElevatorFractionalPosition(), tolerance);
	}

	public boolean isAtPosition(ElevatorMeasure position, ElevatorMeasure tolerance) {
		return isAtPosition(physicalProperties.getFractionalHeight(position), physicalProperties.getFractionalHeight(tolerance));
	}


	public Distance getElevatorHeight() {
		return physicalProperties.convertMotorPositionToPhysicalHeight(getMotorPosition());
	}

	public void stop() {
		motor.stop();
	}

	public void setBrake(boolean brake) {
		motor.setBrake(brake);
	}

	public Double getVoltage() {
		return voltageSignal.getLatestValue();
	}

	public Double getCurrent() {
		return currentSignal.getLatestValue();
	}

	public Rotation2d getMotorPosition() {
		return positionSignal.getLatestValue();
	}

	public double getElevatorFractionalPosition() {
		return physicalProperties.convertMotorPositionToFractionalHeight(getMotorPosition());
	}

	public boolean isBelowPositon(ElevatorMeasure position) {
		return physicalProperties.getFractionalHeight(position) > getElevatorFractionalPosition();
	}

	public boolean isAbovePosition(ElevatorMeasure position) {
		return physicalProperties.getFractionalHeight(position) > getElevatorFractionalPosition();
	}

	public Distance getElevatorPositionMeasure() {
		return physicalProperties.convertMotorPositionToPhysicalHeight(getMotorPosition());
	}

	public Rotation2d getVelocity() {
		return velocitySignal.getLatestValue();
	}

	public void update() {
		motor.updateSimulation();
		motor.updateInputs(voltageSignal, currentSignal, positionSignal, velocitySignal);
		Logger.recordOutput(getLogPath() + "/PositionTarget", targetPosition);
	}

	public void goToPosition(ElevatorMeasure position, ElevatorMeasure changePerSecond) {
		motorGoToPosition(
			physicalProperties.convertFractionalHeightToMotorPosition(physicalProperties.getFractionalHeight(position)),
			physicalProperties.convertFractionalDisplacementToMotorRotation(physicalProperties.getFractionalHeight(changePerSecond))
		);
	}

	private void motorGoToPosition(Rotation2d position, Rotation2d rps) {
		IRequest<Rotation2d> request1 = request.setVelocity(rps).withSetPoint(position);
		motor.applyRequest(request1);
	}

}
