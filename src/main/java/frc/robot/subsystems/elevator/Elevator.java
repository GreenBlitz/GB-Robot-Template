package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.joysticks.Axis;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IFeedForwardRequest;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.Conversions;
import frc.utils.battery.BatteryUtil;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.littletonrobotics.junction.Logger;

public class Elevator extends GBSubsystem {

	protected final ControllableMotor motor;
	private final IFeedForwardRequest positionRequest;
	private final IRequest<Double> voltageRequest;
	private final SysIdCalibrator sysIdCalibrator;
	private final ElevatorCommandBuilder commandsBuilder;
	private final ElevatorSignals elevatorSignals;
	private final double drumDiameter;
	private final double kG;
	private final double minHeight;
	private final double maxHeight;


	public Elevator(
		String logPath,
		ControllableMotor motor,
		IFeedForwardRequest positionRequest,
		IRequest<Double> voltageRequest,
		ElevatorSignals elevatorSignals,
		double kG,
		double drumDiameter,
		double minHeight,
		double maxHeight

	) {
		super(logPath);
		this.motor = motor;
		this.positionRequest = positionRequest;
		this.voltageRequest = voltageRequest;
		this.sysIdCalibrator = new SysIdCalibrator(motor.getSysidConfigInfo(), this, (voltage) -> setVoltage(voltage + getKgVoltage()));
		this.commandsBuilder = new ElevatorCommandBuilder(this);
		setDefaultCommand(commandsBuilder.stayPut());

		this.elevatorSignals = elevatorSignals;
		this.drumDiameter = drumDiameter;
		this.kG = kG;
		this.minHeight = minHeight;
		this.maxHeight = maxHeight;
	}

	public ElevatorCommandBuilder getCommandsBuilder() {
		return this.commandsBuilder;
	}

	public double getVoltage() {
		return this.elevatorSignals.voltageSignal().getLatestValue();
	}

	public double getKgVoltage() {
		return Robot.ROBOT_TYPE.isReal() ? this.kG : 0;
	}

	public void setVoltage(double voltage) {
		this.motor.applyRequest(voltageRequest.withSetPoint(voltage));
	}

	public void stayInPlace() {
		setTargetPosition(this.elevatorSignals.positionSignal().getLatestValue());
	}


	public Rotation2d getPosition() {
		return this.elevatorSignals.positionSignal().getLatestValue();
	}

	public double getMaxHeight() {
		return maxHeight;
	}

	public double getMinHeight() {
		return minHeight;
	}

	public double getDrumDiameter() {
		return drumDiameter;
	}

	public double angleToMeters(Rotation2d angle) {
		return Conversions.angleToDistance(angle, this.drumDiameter);
	}

	public Rotation2d metersToAngle(double distance) {
		return Conversions.distanceToAngle(distance, this.drumDiameter);
	}

	public void setBrake(boolean brake) {
		this.motor.setBrake(brake);
	}

	public boolean isAtPosition(Rotation2d position, Rotation2d tolerance) {
		return this.elevatorSignals.positionSignal().isNear(position, tolerance);
	}

	public boolean isBeforePosition(Rotation2d position) {
		return this.elevatorSignals.positionSignal().isLess(position);
	}

	public boolean isAfterPosition(Rotation2d position) {
		return this.elevatorSignals.positionSignal().isGreater(position);
	}

	public boolean isInSoftLimit() {
		return this.elevatorSignals.positionSignal().isLess(metersToAngle(maxHeight))
			&& this.elevatorSignals.positionSignal().isGreater(this.metersToAngle(minHeight));
	}

	public void stop() {
		this.motor.stop();
	}

	public void resetPosition(double targetPosition) {
		motor.resetPosition(metersToAngle(targetPosition));
	}

	public void resetPosition(Rotation2d targetPosition) {
		motor.resetPosition(targetPosition);
	}

	public void setPower(double power) {
		motor.setPower(power);
	}

	public void setTargetPosition(double position) {
		positionRequest.withSetPoint(this.metersToAngle(position));
	}

	public void setTargetPosition(Rotation2d position) {
		positionRequest.withSetPoint(position);
	}

	public void update() {
		motor.updateSimulation();
		updateInputs();
		log();
	}

	private void updateInputs() {
		motor.updateInputs(elevatorSignals.positionSignal(), elevatorSignals.voltageSignal());
	}

	public void log() {
		Logger.recordOutput(getLogPath() + "/position", elevatorSignals.positionSignal().getLatestValue());
		Logger.recordOutput(getLogPath() + "/voltage", elevatorSignals.voltageSignal().getLatestValue());
	}

	public void setFeedForward(double arbitraryFeedForward) {
		positionRequest.withArbitraryFeedForward(arbitraryFeedForward);
	}

	public void applyCalibrationBindings(SmartJoystick joystick, double maxCalibrationPower) {
		// Calibrate kG using phoenix tuner by setting the voltage

		// Check limits
		joystick.R1.whileTrue(
			commandsBuilder
				.setPower(() -> joystick.getAxisValue(Axis.LEFT_Y) * maxCalibrationPower + (getKgVoltage() / BatteryUtil.getCurrentVoltage()))
		);

		// Calibrate feed forward using sys id:
		sysIdCalibrator.setAllButtonsForCalibration(joystick);
	}

	public void setVoltageWithoutLimit(double voltage) {
		if (voltageRequest instanceof Phoenix6Request<Double> phoenix6VoltageRequest) {
			if (phoenix6VoltageRequest.getControlRequest() instanceof VoltageOut voltageOutRequest) {
				voltageOutRequest.IgnoreSoftwareLimits = true;
				voltageRequest.withSetPoint(voltage);
				motor.applyRequest(voltageRequest);
				voltageOutRequest.IgnoreSoftwareLimits = false;
			}
		}
	}

}
