package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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


    public TalMotorControilMission(int deviceId) {
        this.motor = new TalonFX(deviceId);
        this.deviceId = deviceId;

        this.configurator = this.motor.getConfigurator();

		SoftwareLimitSwitchConfigs softwareConfigs = new SoftwareLimitSwitchConfigs();
		softwareConfigs.withForwardSoftLimitEnable(true);
		softwareConfigs.withForwardSoftLimitThreshold(5);
		softwareConfigs.withReverseSoftLimitEnable(true);
		softwareConfigs.withReverseSoftLimitThreshold(-3);
		this.configurator.apply(softwareConfigs);

        CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
        currentConfigs.StatorCurrentLimitEnable = true;
        currentConfigs.withStatorCurrentLimit(40);
        this.configurator.apply(currentConfigs);

        this.neutralModeValue = NeutralModeValue.Brake;
        this.rotationDirection = InvertedValue.Clockwise_Positive;
        this.lastPos = this.getPosition();
        this.mistakeSum = this.mistakeFromPos();
    }

    public void invertRotation() {
        MotorOutputConfigs configs = new MotorOutputConfigs();
        this.rotationDirection = rotationDirection == InvertedValue.Clockwise_Positive ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
        this.rotationDirection = InvertedValue.values()[1 - this.rotationDirection.ordinal()];
        configs.withInverted(this.rotationDirection);
        this.configurator.apply(configs);
    }

    public void changeNeutralMode() {
        this.neutralModeValue = NeutralModeValue.values()[1 - this.neutralModeValue.ordinal()];
        this.motor.setNeutralMode(this.neutralModeValue);
    }

    public void setPoisition() {
        this.configurator.setPosition(2);
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


    public double pidToTargetPos() {
        this.mistakeSum += this.mistakeFromPos();
        double p = TalMotorControilMissionConstants.KP * this.mistakeFromPos();
        double i = TalMotorControilMissionConstants.KI * this.mistakeSum;
        double d = TalMotorControilMissionConstants.KD * ((this.lastPos - this.getPosition()) / TimeUtil.getLatestCycleTimeSeconds());
        this.lastPos = this.getPosition();
        return p + i + d;
    }

    public double mistakeFromPos() {
        return TalMotorControilMissionConstants.wantedPos - this.getPosition();
    }

    public void stopMotor() {
        this.motor.stopMotor();
    }

    public double getPosition() {
        return this.motor.getPosition().getValueAsDouble();
    }

    public double getSpeed() {
        return this.motor.get();
    }

    public double getVoltage() {
        return this.motor.getMotorVoltage().getValueAsDouble();
    }

    public double getCurrent() {
        return this.motor.getMotorStallCurrent().getValueAsDouble();
    }

    public void logUpdates() {
        Logger.recordOutput("Position", this.getPosition());
        Logger.recordOutput("Speed", this.getSpeed());
        Logger.recordOutput("Voltage", this.getVoltage());
        Logger.recordOutput("Current", this.getCurrent());
        Logger.recordOutput("Connected", this.motor.isConnected());
    }


}
