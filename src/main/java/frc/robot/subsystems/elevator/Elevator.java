package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.joysticks.Axis;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IFeedForwardRequest;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
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
	private final ElevatorCommandsBuilder commandsBuilder;
	private final ElevatorSignals elevatorSignals;
	private final double drumDiameterMeters;
	private final double kG;
	private final double minHeightMeters;
	private final double maxHeightMeters;

	public Elevator(
		String logPath,
		ControllableMotor motor,
		IFeedForwardRequest positionRequest,
		IRequest<Double> voltageRequest,
		ElevatorSignals elevatorSignals,
		double kG,
		double drumDiameterMeters,
		double minHeightMeters,
		double maxHeightMeters
	) {
		super(logPath);
		this.motor = motor;
		this.positionRequest = positionRequest;
		this.voltageRequest = voltageRequest;
		this.sysIdCalibrator = new SysIdCalibrator(motor.getSysidConfigInfo(), this, (voltage) -> setVoltage(voltage + getKgVoltage()));
		this.commandsBuilder = new ElevatorCommandsBuilder(this);
		setDefaultCommand(commandsBuilder.stayInPlace());
		this.elevatorSignals = elevatorSignals;
		this.drumDiameterMeters = drumDiameterMeters;
		this.kG = kG;
		this.minHeightMeters = minHeightMeters;
		this.maxHeightMeters = maxHeightMeters;
	}

	public ElevatorCommandsBuilder getCommandsBuilder() {
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
		setTargetPosition(rotationsToMeters(this.elevatorSignals.positionSignal().getLatestValue()));
	}

	public double getPosition() {
		return rotationsToMeters(this.elevatorSignals.positionSignal().getLatestValue());
	}

	public double getMaxHeightMeters() {
		return maxHeightMeters;
	}

	public double getMinHeightMeters() {
		return minHeightMeters;
	}

	public double getDrumDiameterMeters() {
		return drumDiameterMeters;
	}

	public double rotationsToMeters(Rotation2d angle) {
		return Conversions.angleToDistance(angle, this.drumDiameterMeters);
	}

	public Rotation2d metersToRotations(double distance) {
		return Conversions.distanceToAngle(distance, this.drumDiameterMeters);
	}

	public void setBrake(boolean brake) {
		this.motor.setBrake(brake);
	}

	public boolean isAtPosition(double position, double tolerance) {
		return this.elevatorSignals.positionSignal().isNear(metersToRotations(position), metersToRotations(tolerance));
	}

	public boolean isBeforePosition(double position) {
		return this.elevatorSignals.positionSignal().isLess(metersToRotations(position));
	}

	public boolean isAfterPosition(double position) {
		return this.elevatorSignals.positionSignal().isGreater(metersToRotations(position));
	}

	public boolean isInMechanicalLimit() {
		return this.elevatorSignals.positionSignal().isLess(metersToRotations(maxHeightMeters))
			&& this.elevatorSignals.positionSignal().isGreater(metersToRotations(minHeightMeters));
	}

	public void stop() {
		this.motor.stop();
	}

	public void resetPosition(double positionMeters) {
		motor.resetPosition(metersToRotations(positionMeters));
	}

	public void setPower(double power) {
		motor.setPower(power);
	}

	public void setTargetPosition(double positionMeters) {
		positionRequest.withSetPoint(this.metersToRotations(positionMeters));
	}

	public void update() {
		motor.updateSimulation();
		updateInputs();
		log();
	}

	private void updateInputs() {
		motor.updateInputs(elevatorSignals.positionSignal(), elevatorSignals.voltageSignal());
        for (InputSignal<?> signal : elevatorSignals.otherSignals()) {
            motor.updateInputs(signal);
        }
	}

	public void log() {
		Logger.recordOutput(getLogPath() + "/positionMeters", elevatorSignals.positionSignal().getLatestValue());
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
