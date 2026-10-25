package frc.robot.subsystems.tal;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.battery.BatteryUtil;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ModuleTal extends GBSubsystem {

	private final TalonFX driveMotor;
	private final TalonFX steerMotor;

	private final TalonFXConfiguration steerConfigs;
	private final TalonFXConfiguration driveConfigs;
	private final TalonFXConfigurator steerConfigurator;
	private final TalonFXConfigurator driveConfigurator;

	private InvertedValue steerRotationDirection;
	private InvertedValue driveRotationDirection;
	private NeutralModeValue driveNeutralModeValue;
	private NeutralModeValue steerNeutralModeValue;

	private Supplier<Double> targetMotorPower;
	private Supplier<Double> targetJoystickAngle;

	private ModulePowers modulePowers;
	private final PositionVoltage request;

	private ModuleTalCommandBuilder commandBuilder = new ModuleTalCommandBuilder(this);

	private SimpleMotorSimulation steerSimulation;
	private SimpleMotorSimulation driveSimulation;
	private TalonFXSimState steerMotorSimState;
	private TalonFXSimState driveMotorSimState;

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
		this.targetJoystickAngle = () -> (Double) 0.0;
		this.targetMotorPower = () -> (Double) 0.0;
		this.setAmpereLimit(ModuleTalConstants.ampereLimit);
		this.commandBuilder = new ModuleTalCommandBuilder(this);
		this.request = new PositionVoltage(0).withSlot(0);

		if (Robot.ROBOT_TYPE.isSimulation()) {
			this.driveSimulation = new SimpleMotorSimulation(
				new DCMotorSim(
					LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.0000001, ModuleTalConstants.driveGearRatio),
					DCMotor.getKrakenX60(1)
				)
			);
			this.steerSimulation = new SimpleMotorSimulation(
				new DCMotorSim(
					LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.0000001, ModuleTalConstants.steerGearRatio),
					DCMotor.getKrakenX60(1)
				)
			);

			this.steerMotorSimState = steerMotor.getSimState();
			steerMotorSimState.setSupplyVoltage(BatteryUtil.DEFAULT_VOLTAGE);

			this.driveMotorSimState = driveMotor.getSimState();
			driveMotorSimState.setSupplyVoltage(BatteryUtil.DEFAULT_VOLTAGE);
		}
	}

	private void configureSteerMotor() {
		this.steerConfigs.Slot0.kP = ModuleTalConstants.kPSteer;
		this.steerConfigs.Slot0.kI = ModuleTalConstants.kISteer;
		this.steerConfigs.Slot0.kD = ModuleTalConstants.kDSteer;
		this.steerConfigs.Feedback.SensorToMechanismRatio = ModuleTalConstants.steerGearRatio;
		this.steerMotor.setNeutralMode(this.steerNeutralModeValue);
		this.steerConfigurator.apply(this.steerConfigs);
	}

	private void configureDriveMotor() {
		this.driveConfigs.Slot0.kP = ModuleTalConstants.kPDrive;
		this.driveConfigs.Slot0.kI = ModuleTalConstants.kIDrive;
		this.driveConfigs.Slot0.kD = ModuleTalConstants.kDDrive;
		this.driveConfigs.Feedback.SensorToMechanismRatio = ModuleTalConstants.driveGearRatio;
		this.driveMotor.setNeutralMode(this.driveNeutralModeValue);
		this.driveConfigurator.apply(this.driveConfigs);
	}

	public void setDriveAmpereLimit(double ampereLimit) {
		this.driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
		this.driveConfigs.CurrentLimits.withStatorCurrentLimit(ampereLimit);
		this.driveConfigurator.apply(this.driveConfigs);
	}

	public void setSteerAmpereLimit(double ampereLimit) {
		this.steerConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
		this.steerConfigs.CurrentLimits.withStatorCurrentLimit(ampereLimit);
		this.steerConfigurator.apply(this.steerConfigs);
	}

	public void setAmpereLimit(double ampereLimit) {
		this.setSteerAmpereLimit(ampereLimit);
		this.setDriveAmpereLimit(ampereLimit);
	}

	public ModuleTalCommandBuilder getCommandBuilder() {
		return this.commandBuilder;
	}

	public void changeDriveNeutralMode() {
		this.driveNeutralModeValue = this.driveNeutralModeValue == NeutralModeValue.Brake ? NeutralModeValue.Coast : NeutralModeValue.Brake;
		this.driveMotor.setNeutralMode(this.driveNeutralModeValue);
	}

	public void changeSteerNeutralMode() {
		this.steerNeutralModeValue = this.steerNeutralModeValue == NeutralModeValue.Brake ? NeutralModeValue.Coast : NeutralModeValue.Brake;
		this.steerMotor.setNeutralMode(this.steerNeutralModeValue);
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

	public void changeNeutralModeMotors() {
		this.changeDriveNeutralMode();
		this.changeSteerNeutralMode();
	}

	public void drive(double angle, double power) {
		steerToAngle(angle);
		setDrivePower(power);
	}


	public void steerToAngle(double angle) {
		this.steerMotor.setControl(request.withPosition(angle));
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

	public double getDrivePosition() {
		return this.driveMotor.getPosition().getValueAsDouble();
	}

	@Override
	protected void subsystemPeriodic() {
		if (Robot.ROBOT_TYPE.isSimulation()) {
			updateSteerSim();
			updateDriveSim();
		}
		Logger.recordOutput(getLogPath() + "/currentDriveSpeed", getDriveSpeed());
		Logger.recordOutput(getLogPath() + "/targetDrivePower", getTargetMotorPower().get());
		Logger.recordOutput(getLogPath() + "/driveVoltage", getDriveVoltage());
		Logger.recordOutput(getLogPath() + "/wheelAngle", this.getSteerAngle());
		Logger.recordOutput(getLogPath() + "/targetWheelAngle", this.targetJoystickAngle.get());
	}

	public void update() {
		this.targetMotorPower = () -> (Double) Math.sqrt(Math.pow(this.modulePowers.xPower, 2) + Math.pow(this.modulePowers.yPower, 2));
		this.targetJoystickAngle = this::calculateAngle;
	}

	public Supplier<Double> getTargetJoystickAngle() {
		return targetJoystickAngle;
	}

	protected Double calculateAngle() {
		return (Double) Math.toDegrees(Math.atan2(this.modulePowers.yPower, this.modulePowers.xPower));
	}

	protected void updateSteerSim() {
		steerSimulation.setInputVoltage(steerMotorSimState.getMotorVoltage());
		steerSimulation.updateMotor();
		steerMotorSimState.setRawRotorPosition(steerSimulation.getRotorPosition().getRotations());
		steerMotorSimState.setRotorAcceleration(steerSimulation.getRotorVelocityRPS().getRotations());
	}

	protected void updateDriveSim() {
		driveSimulation.setInputVoltage(driveMotorSimState.getMotorVoltage());
		driveSimulation.updateMotor();
		driveMotorSimState.setRawRotorPosition(driveSimulation.getRotorPosition().getRotations());
		driveMotorSimState.setRotorAcceleration(driveSimulation.getRotorVelocityRPS().getRotations());
	}

}
