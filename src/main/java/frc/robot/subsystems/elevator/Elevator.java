package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.cansparkmax.SparkMaxAngleRequest;
import frc.robot.subsystems.elevator.factories.RealElevatorConstants;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Elevator extends GBSubsystem {

	private final DigitalInputInputsAutoLogged digitalInputsInputs;
	private final ElevatorCommandBuilder commandBuilder;
	private final SparkMaxAngleRequest angleRequest;

	private final ElevatorStuff elevatorStuff;
	private final ControllableMotor mainMotor;
	private final IDigitalInput limitSwitch;

	public Elevator(ElevatorStuff elevatorStuff) {
		super(elevatorStuff.logPath());

		this.mainMotor = elevatorStuff.mainMotor();
		this.limitSwitch = elevatorStuff.digitalInput();

		this.digitalInputsInputs = new DigitalInputInputsAutoLogged();
		this.elevatorStuff = elevatorStuff;
		this.commandBuilder = new ElevatorCommandBuilder(this);
		this.angleRequest = elevatorStuff.angleRequest();
	}

	public static double ElevatorFeedforward(CANSparkMax motor) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealElevatorConstants.FEEDFORWARD_CALCULATOR.calculate(motor.getEncoder().getVelocity());
			case SIMULATION -> 0.0;
		};
	}

	public ElevatorCommandBuilder getCommandBuilder() {
		return commandBuilder;
	}

	public void setPower(double power) {
		mainMotor.setPower(power);
	}

	public void stop() {
		elevatorStuff.mainMotor().stop();
	}

	public void setBrake(boolean brake) {
		elevatorStuff.mainMotor().setBrake(brake);
	}

	public void setTargetAngle(Rotation2d angle) {
		mainMotor.applyAngleRequest(angleRequest.withSetPoint(angle));
	}

	public boolean isAtBackwardLimit() {
		return digitalInputsInputs.debouncedValue;
	}

	public void stayInPlace() {
		mainMotor.applyAngleRequest(angleRequest.withSetPoint(elevatorStuff.mainMotorPositionSignal().getLatestValue()));
	}

	public void updateInputs() {
		limitSwitch.updateInputs(digitalInputsInputs);
		mainMotor.updateSignals(elevatorStuff.mainMotorPositionSignal(), elevatorStuff.voltageSignal());
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		Logger.processInputs(elevatorStuff.digitalInputsLogPath(), digitalInputsInputs);
	}

}
