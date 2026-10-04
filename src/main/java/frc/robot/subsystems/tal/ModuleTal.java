package frc.robot.subsystems.tal;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class ModuleTal extends GBSubsystem {

	private TalonFX driveMotor;
	private TalonFX steerMotor;
	private TalonFXConfiguration steerConfigs;
	private TalonFXConfiguration driveConfigs;
	private TalonFXConfigurator steerConfigurator;
	private TalonFXConfigurator driveConfigurator;
	private InvertedValue steerRotationDirection;
	private InvertedValue driveRotationDirection;
	private NeutralModeValue driveNeutralModeValue;
	private NeutralModeValue steerNeutralModeValue;
	private double targetMotorPower;
	private double targetAngle;

	public ModuleTal(String logPath, int driveID, int steerID) {
		super(logPath);
		this.driveMotor = new TalonFX(driveID);
		this.steerMotor = new TalonFX(steerID);
		this.driveConfigurator = driveMotor.getConfigurator();
		this.steerConfigurator = steerMotor.getConfigurator();
		this.steerConfigs = new TalonFXConfiguration();
		this.driveConfigs = new TalonFXConfiguration();
		this.steerNeutralModeValue = NeutralModeValue.Brake;
		this.driveNeutralModeValue = NeutralModeValue.Brake;
		configureSteerMotor();
		this.targetAngle = 0;
		this.targetMotorPower = 0;
	}

	private void configureSteerMotor() {
		this.steerConfigs.ClosedLoopGeneral.withContinuousWrap(true);
		this.steerConfigurator.apply(this.steerConfigs);
	}

	public void setAmpereLimit(double ampereLimit) {
		this.steerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
		this.steerConfigs.CurrentLimits.withStatorCurrentLimit(ampereLimit);
		this.steerConfigurator.apply(this.steerConfigs);
		this.driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
		this.driveConfigs.CurrentLimits.withStatorCurrentLimit(ampereLimit);
		this.driveConfigurator.apply(this.driveConfigs);
	}

	public void changeDriveNeutralMode() {
		this.driveNeutralModeValue = this.driveNeutralModeValue == NeutralModeValue.Brake ? NeutralModeValue.Coast : NeutralModeValue.Brake;
	}

	public void changeSteerNeutralMode() {
		this.steerNeutralModeValue = this.steerNeutralModeValue == NeutralModeValue.Brake ? NeutralModeValue.Coast : NeutralModeValue.Brake;
	}

	protected void invertDriveRotation() {
		this.driveRotationDirection = this.driveRotationDirection == InvertedValue.Clockwise_Positive
			? InvertedValue.CounterClockwise_Positive
			: InvertedValue.Clockwise_Positive;
		this.driveConfigs.MotorOutput.withInverted(this.driveRotationDirection);
		this.driveConfigurator.apply(this.driveConfigs);
	}

	protected void invertSteerRotation() {
		this.steerRotationDirection = this.steerRotationDirection == InvertedValue.Clockwise_Positive
			? InvertedValue.CounterClockwise_Positive
			: InvertedValue.Clockwise_Positive;
		this.steerConfigs.MotorOutput.withInverted(this.steerRotationDirection);
		this.steerConfigurator.apply(this.steerConfigs);
	}

	public void invertMotors() {
		this.invertDriveRotation();
		this.invertSteerRotation();
	}

	public void steerToAngle(double targetAngle) {
		this.targetAngle = targetAngle;
		this.steerMotor.setControl(new PositionDutyCycle(targetAngle));
	}

	public double getDriveVoltage() {
		return this.driveMotor.getMotorVoltage().getValueAsDouble();
	}

	public void setDriveVoltage(double voltage) {
		this.driveMotor.setVoltage(voltage);
	}

	public void setDrivePower(double power) {
		this.targetMotorPower = power;
		this.driveMotor.set(power);
	}

	public void stop() {
		this.driveMotor.stopMotor();
		this.steerMotor.stopMotor();
	}

	public double getSteerAngle() {
		return this.steerMotor.getPosition().getValueAsDouble();
	}

	public double getDriveSpeed() {
		return this.driveMotor.get();
	}


	public boolean isDriveAtSpeed(double speed) {
		return this.driveMotor.get() == speed;
	}

	public boolean isDriveAtAngle(double angle) {
		return this.getSteerAngle() == angle;
	}

	@Override
	protected void subsystemPeriodic() {
		Logger.recordOutput(getLogPath() + "/currentDriveSpeed", getDriveSpeed());
		Logger.recordOutput(getLogPath() + "/targetDrivePower", this.targetMotorPower);
		Logger.recordOutput(getLogPath() + "/driveVoltage", getDriveVoltage());
		Logger.recordOutput(getLogPath() + "/wheelAngle", this.getSteerAngle());
		Logger.recordOutput(getLogPath() + "/targetWheelAngle", this.targetAngle);
	}


}
