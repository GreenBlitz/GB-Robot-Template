package frc.robot.subsystems.tal;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

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
	private Supplier<Double> targetMotorPower;
	private Supplier<Double> targetAngle;
	private ModulePowers modulePowers;
	private PositionVoltage request;
	private Supplier<Double> previousAngle;
	private ModuleTalCommandBuilder commandBuilder = new ModuleTalCommandBuilder(this);


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
		configureDriveMotor();
		this.targetAngle = () -> 0.0;
		this.targetMotorPower = () -> 0.0;
		this.previousAngle = () -> 0.0;
		this.setAmpereLimit(40);
		this.commandBuilder = new ModuleTalCommandBuilder(this);
		this.request = new PositionVoltage(0).withSlot(0);
	}


	private void configureSteerMotor() {
		this.steerConfigs.Slot0.kP = 0.04;
		this.steerConfigs.Slot0.kI = 0.02;
		this.steerConfigs.Slot0.kD = 0.01;
		this.steerConfigs.Feedback.SensorToMechanismRatio = 1 / 12.8;
		this.steerConfigurator.apply(this.steerConfigs);
	}

	private void configureDriveMotor() {
		this.driveConfigs.Slot0.kP = 0.5;
		this.driveConfigs.Slot0.kI = 0;
		this.driveConfigs.Slot0.kD = 0;
		this.driveConfigs.Feedback.SensorToMechanismRatio = 1 / 8.14;
		this.driveConfigurator.apply(this.driveConfigs);
	}

	public void setAmpereLimit(double ampereLimit) {
		this.steerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
		this.steerConfigs.CurrentLimits.withStatorCurrentLimit(ampereLimit);
		this.steerConfigurator.apply(this.steerConfigs);
		this.driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
		this.driveConfigs.CurrentLimits.withStatorCurrentLimit(ampereLimit);
		this.driveConfigurator.apply(this.driveConfigs);
	}

	public ModuleTalCommandBuilder getCommandBuilder() {
		return this.commandBuilder;
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

	public void drive(double targetAngle, double power) {
		steerToAngle();
		setDrivePower(power);
	}

	public void steerToAngle() {
		if (this.previousAngle.get() * this.targetAngle.get() < 0) {
			this.steerConfigurator.setPosition(this.previousAngle.get() * -1);
		}
		this.steerMotor.setControl(request.withPosition(this.targetAngle.get()));
	}

	public double getDriveVoltage() {
		return this.driveMotor.getMotorVoltage().getValueAsDouble();
	}

	public void setDriveVoltage(double voltage) {
		this.driveMotor.setVoltage(voltage);
	}

	public void setDrivePower(double power) {
		this.driveMotor.set(power);
	}

	public void stopSteer() {
		this.steerMotor.stopMotor();
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

	public void setModulePowers(ModulePowers modulePowers) {
		this.modulePowers = modulePowers;
	}


	public boolean isDriveAtSpeed(double speed) {
		return this.driveMotor.get() == speed;
	}

	public boolean isDriveAtAngle(double angle) {
		return this.getSteerAngle() == angle;
	}

	public Supplier<Double> getTargetMotorPower() {
		return targetMotorPower;
	}

	@Override
	protected void subsystemPeriodic() {
		Logger.recordOutput(getLogPath() + "/currentDriveSpeed", getDriveSpeed());
		Logger.recordOutput(getLogPath() + "/targetDrivePower", getTargetMotorPower().get());
		Logger.recordOutput(getLogPath() + "/driveVoltage", getDriveVoltage());
		Logger.recordOutput(getLogPath() + "/wheelAngle", this.getSteerAngle());
		Logger.recordOutput(getLogPath() + "/targetWheelAngle", this.targetAngle.get());
		Logger.recordOutput(getLogPath() + "/previousAngle", this.previousAngle.get());
	}

	public void update() {
		this.targetMotorPower = () -> Math.sqrt(Math.pow(this.modulePowers.xPower, 2) + Math.pow(this.modulePowers.yPower, 2));
		this.previousAngle = this.targetAngle;
		this.targetAngle = this::calculateAngle;
	}

	public boolean isCloseToAngle(double target) {
		if (this.getSteerAngle() - target <= 1 && this.getSteerAngle() - target >= -1) {
			Logger.recordOutput(getLogPath() + "/isclose", this.getSteerAngle() - target);
			return true;
		}
		return false;
	}

	public Supplier<Double> getTargetAngle() {
		return targetAngle;
	}

	private double calculateAngle() {
		return Math.atan2(this.modulePowers.yPower, this.modulePowers.xPower) * 50;
	}

}
