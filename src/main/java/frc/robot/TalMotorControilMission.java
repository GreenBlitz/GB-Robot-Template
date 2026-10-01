package frc.robot;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.*;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;


public class TalMotorControilMission {

	private TalonFX motor;
	private final int deviceId;
	private TalonFXConfigurator configurator;
	private InvertedValue rotationDirection;
	private NeutralModeValue neutralModeValue;
	private double lastPos;
	private double mistakeSum;
	private StatusSignal<Voltage> motorVoltage;
	private Measure<AngleUnit> motorPositionLatencyCompincated;
	private StatusSignal<Angle> motorPosition;
	private StatusSignal<Current> motorCurrent;


	public TalMotorControilMission(int deviceId) {
		this.motor = new TalonFX(deviceId, CANBus.roboRIO());
		this.deviceId = deviceId;

		this.motorVoltage = this.getVoltage();
		this.motorCurrent = this.getCurrent();
		this.motorPosition = this.getPosition();
		this.motorPositionLatencyCompincated = this.getPositionLatencyCompincated();

		this.configurator = this.motor.getConfigurator();

		this.neutralModeValue = NeutralModeValue.Brake;
		this.rotationDirection = InvertedValue.Clockwise_Positive;
		this.lastPos = this.getPositionAsDouble();
		this.mistakeSum = this.mistakeFromPos();
		this.motor.optimizeBusUtilization();
	}

	public void pidWithPositionVoltage() {
		TalonFXConfiguration pidConfigs = new TalonFXConfiguration();
		pidConfigs.Slot0.kP = 2;
		pidConfigs.Slot0.kI = 0.5;
		pidConfigs.Slot0.kD = 2;
		PositionVoltage positionVoltage = new PositionVoltage(5);
		this.motor.setControl(positionVoltage);
	}

	private void setSoftwareConfigs() {
		SoftwareLimitSwitchConfigs softwareConfigs = new SoftwareLimitSwitchConfigs();
		softwareConfigs.withForwardSoftLimitEnable(true);
		softwareConfigs.withForwardSoftLimitThreshold(5);
		softwareConfigs.withReverseSoftLimitEnable(true);
		softwareConfigs.withReverseSoftLimitThreshold(-3);
		this.configurator.apply(softwareConfigs);
	}

	private void setCurrentConfigs() {
		CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
		currentConfigs.StatorCurrentLimitEnable = true;
		currentConfigs.withStatorCurrentLimit(40);
		this.configurator.apply(currentConfigs);
	}

	public void invertRotation() {
		MotorOutputConfigs configs = new MotorOutputConfigs();
		this.rotationDirection = rotationDirection == InvertedValue.Clockwise_Positive
			? InvertedValue.CounterClockwise_Positive
			: InvertedValue.Clockwise_Positive;
		this.rotationDirection = InvertedValue.values()[1 - this.rotationDirection.ordinal()];
		configs.withInverted(this.rotationDirection);
		this.configurator.apply(configs);
	}

	public void changeNeutralMode() {
		this.neutralModeValue = NeutralModeValue.values()[1 - this.neutralModeValue.ordinal()];
		this.motor.setNeutralMode(this.neutralModeValue);
	}

	public void setPoisition() {
		this.configurator.setPosition(200);
	}

	public void MoveForwardHalfPower() {
		this.motor.set(0.5);
	}

	public void MoveBackwardsHalfPower() {
		this.motor.set(-0.5);
	}

	public void MoveForwardTenthPower() {
		this.motor.set(0.1);
	}

	public void runPID() {
		this.motor.setVoltage(pidToTargetPos());
	}

	public void refreshAllSignals() {
		this.motorCurrent.refresh();
		this.motorPosition = this.getPosition();
		this.motorVoltage.refresh();
	}

	public double pidToTargetPos() {
		this.mistakeSum = this.mistakeFromPos();
		Logger.recordOutput("mistakeSum", this.mistakeSum);
		double p = TalMotorControilMissionConstants.KP * this.mistakeFromPos();
		double i = TalMotorControilMissionConstants.KI * this.mistakeSum;
		double d = TalMotorControilMissionConstants.KD * ((this.lastPos - this.getPositionAsDouble()) / TimeUtil.getLatestCycleTimeSeconds());
		this.lastPos = this.getPositionAsDouble();
		Logger.recordOutput("lastPos", this.lastPos);
		Logger.recordOutput("PID", p + i + d);
		return p + i + d;
	}

	public double mistakeFromPos() {
		return TalMotorControilMissionConstants.wantedPos - this.getPositionAsDouble();
	}

	public void stopMotor() {
		this.motor.stopMotor();
	}

	public StatusSignal<Angle> getPosition() {
		StatusSignal<Angle> positionSignal = this.motor.getPosition();
		positionSignal.setUpdateFrequency(50);
		return positionSignal;
	}

	public Measure<AngleUnit> getPositionLatencyCompincated() {
		StatusSignal<AngularVelocity> velocity = motor.getVelocity();
		this.motorPosition.getTimestamp().getLatency();
		return BaseStatusSignal.getLatencyCompensatedValue(this.motorPosition, velocity);
	}

	public double getSpeed() {
		return this.motor.get();
	}

	public StatusSignal<Voltage> getVoltage() {
		StatusSignal<Voltage> voltageSignal = this.motor.getMotorVoltage();
		voltageSignal.setUpdateFrequency(50);
		return voltageSignal;
	}

	public StatusSignal<Current> getCurrent() {
		StatusSignal<Current> currentSignal = this.motor.getMotorStallCurrent();
		currentSignal.setUpdateFrequency(50);
		return currentSignal;
	}

	public double getPositionAsDouble() {
		return this.motor.getPosition().getValueAsDouble();
	}


	public double getVoltageAsDouble() {
		return this.motor.getMotorVoltage().getValueAsDouble();
	}

	public double getCurrentAsDouble() {
		return this.motor.getMotorStallCurrent().getValueAsDouble();
	}


	public void logUpdates() {
		Logger.recordOutput("Position", this.getPositionAsDouble());
		Logger.recordOutput("Speed", this.getSpeed());
		Logger.recordOutput("Voltage", this.getVoltageAsDouble());
		Logger.recordOutput("Current", this.getCurrentAsDouble());
		Logger.recordOutput("Connected", this.motor.isConnected());
	}


}
