package frc;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.signals.InvertedValue;


public class AlonFX {

	private final TalonFX motor;
	private InvertedValue direction;
	private final String logPath;

	StatusSignal<AngularVelocity> velocity;
	StatusSignal<Voltage> voltage;
	StatusSignal<Current> current;
	StatusSignal<Angle> position;

	public AlonFX(int deviceId, CANBus canBus, String logPath, double gearRatio, double kP) {
		this.logPath = logPath;
		this.motor = new TalonFX(deviceId, canBus);
		direction = InvertedValue.CounterClockwise_Positive;
		TalonFXConfiguration configuration = new TalonFXConfiguration();

		SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();

		softwareLimitSwitchConfigs.ForwardSoftLimitEnable = false;
		softwareLimitSwitchConfigs.ReverseSoftLimitEnable = false;
		configuration.SoftwareLimitSwitch = softwareLimitSwitchConfigs;

		CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
		currentLimitsConfigs.StatorCurrentLimitEnable = true;
		currentLimitsConfigs.SupplyCurrentLowerLimit = 5;
		currentLimitsConfigs.StatorCurrentLimit = 40;
		configuration.CurrentLimits = currentLimitsConfigs;

		Slot0Configs slot0Configs = new Slot0Configs();
		slot0Configs.kP = 1.7197265625;
		slot0Configs.kD = 0.0001;
		slot0Configs.kI = 0;
		configuration.Slot0 = slot0Configs;
		MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs().withInverted(direction);
		configuration.MotorOutput = motorOutputConfigs;

		FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
		feedbackConfigs.SensorToMechanismRatio = gearRatio;
		configuration.Feedback = feedbackConfigs;

		motor.getConfigurator().apply(configuration);
		motor.optimizeBusUtilization();

		velocity = motor.getVelocity();
		velocity.setUpdateFrequency(50);
		voltage = motor.getMotorVoltage();
		voltage.setUpdateFrequency(50);
		current = motor.getStatorCurrent();
		current.setUpdateFrequency(50);
		position = motor.getPosition();
		position.setUpdateFrequency(50);
	}

	private boolean isMotorConnected() {
		return motor.isConnected();
	}

	public void logMotorConnection() {
		Logger.recordOutput(logPath + "/isMotorConnected", isMotorConnected());
	}

	public void stopMotor() {
		motor.stopMotor();
	}

	public void setPower(double amount) {
		motor.set(amount);
	}


	public void driveToPositionTick(double angleRadians) {
		double difference = angleRadians - getPosition().getRadians();
		setVoltage(difference / (2 * Math.PI));
		Logger.recordOutput(logPath + "/target", angleRadians);
		Logger.recordOutput(logPath + "/positionInRadians", getPosition().getRadians());
	}

	public void driveToPosition(double positionRadians) {
		PositionVoltage positionVoltage = new PositionVoltage(positionRadians / (2 * Math.PI));
		motor.setControl(positionVoltage);
	}


	public void moveAtHalfPower() {
		setPower(0.5);
	}

	public void moveReverseTenthSpeed() {
		setPower(-0.1);
	}

	public Rotation2d getPosition() {
		position.refresh();
		return Rotation2d.fromRadians(StatusSignal.getLatencyCompensatedValue(position, velocity).baseUnitMagnitude());
	}

	public Rotation2d getVelocity() {
		velocity.refresh();
		return Rotation2d.fromRotations(velocity.getValueAsDouble());
	}

	public double getVoltage() {
		voltage.refresh();
		return voltage.getValueAsDouble();
	}

	public double getCurrent() {
		current.refresh();
		return current.getValueAsDouble();
	}

	public void invertMotor() {
		MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
		motorOutputConfigs.withInverted(getMotorInvertedDirection());
		motor.getConfigurator().apply(motorOutputConfigs);
		direction = getMotorInvertedDirection();
	}

	public void setVoltage(double voltage) {
		motor.setVoltage(voltage);
	}

	public void setNeutralMode(NeutralModeValue neutralMode) {
		MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
		motorOutputConfigs.withNeutralMode(neutralMode);
		motor.getConfigurator().apply(motorOutputConfigs);
	}

	public void setPosition(double angle) {
		motor.setPosition(angle);
	}

	public void setPosition(Angle angle) {
		motor.setPosition(angle);
	}

	private InvertedValue getMotorInvertedDirection() {
		return direction == InvertedValue.Clockwise_Positive ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
	}

	public void runInStablePowerToPosition(double power) {
		setPower(power);
	}


	public void logAll() {
		Logger.recordOutput(logPath + "/position", position.refresh().getValue());
		Logger.recordOutput(logPath + "/velocity", velocity.refresh().getValue());
		Logger.recordOutput(logPath + "/voltage", voltage.refresh().getValue());
		Logger.recordOutput(logPath + "/current", current.refresh().getValue());
		logMotorConnection();
	}

	public double getPIDTarget() {
		return motor.getClosedLoopReference().getValueAsDouble();
	}

}
