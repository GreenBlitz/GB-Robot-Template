package frc;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.RobotConstants;
import frc.robot.hardware.interfaces.IMotor;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.mechanisms.MechanismSimulation;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.utils.battery.BatteryUtil;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.signals.InvertedValue;

public class AlonFX implements IMotor {

	private final TalonFX motor;
	private InvertedValue direction;
	private final String logpath;
	private SimpleMotorSimulation simulation;

	private final StatusSignal<AngularVelocity> velocity;
	private final StatusSignal<Voltage> voltage;
	private final StatusSignal<Current> current;
	private final StatusSignal<Angle> position;

	public AlonFX(int deviceId, CANBus canBus, String logPath, double gearRatio, double kP, double momentOfInertiaKgM2) {
		this.logpath = logPath;
		this.motor = new TalonFX(deviceId, canBus);
		TalonFXConfiguration configuration = createConfig(gearRatio, kP);
		motor.getConfigurator().apply(configuration);
		velocity = motor.getVelocity();
		velocity.setUpdateFrequency(RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		voltage = motor.getMotorVoltage();
		voltage.setUpdateFrequency(RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		current = motor.getStatorCurrent();
		current.setUpdateFrequency(RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		position = motor.getPosition();
		position.setUpdateFrequency(RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		motor.optimizeBusUtilization();
		if (RobotManager.isSimulation()){
			simulation=new SimpleMotorSimulation(
					new DCMotorSim(
							LinearSystemId.createDCMotorSystem(
									DCMotor.getKrakenX60(1),
									momentOfInertiaKgM2,
									gearRatio
							),
							DCMotor.getKrakenX60Foc(1)
					)
			);
		}
		motor.getSimState().setSupplyVoltage(BatteryUtil.DEFAULT_VOLTAGE);
		motor.getSimState().setMotorType(TalonFXSimState.MotorType.KrakenX60);
	}

	private TalonFXConfiguration createConfig(double gearRatio, double kP) {
		direction = InvertedValue.CounterClockwise_Positive;
		TalonFXConfiguration configuration = new TalonFXConfiguration();

		CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
		currentLimitsConfigs.StatorCurrentLimitEnable = true;
		currentLimitsConfigs.SupplyCurrentLowerLimit = 5;
		currentLimitsConfigs.StatorCurrentLimit = 40;
		configuration.CurrentLimits = currentLimitsConfigs;

		Slot0Configs slot0Configs = new Slot0Configs();
		slot0Configs.kP = kP;
		slot0Configs.kD = 0.0001;
		slot0Configs.kI = 0;
		configuration.Slot0 = slot0Configs;

		MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
		motorOutputConfigs.Inverted = direction;
		configuration.MotorOutput = motorOutputConfigs;

		FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
		feedbackConfigs.SensorToMechanismRatio = gearRatio;
		configuration.Feedback = feedbackConfigs;
		return configuration;
	}

	public void stop() {
		motor.stopMotor();
		//motor.set(0);
	}

	@Override
	public void updateSimulation() {
		simulation.updateMotor();
		motor.getSimState().setRawRotorPosition(simulation.getRotorPosition().getRotations());
		motor.getSimState().setRotorVelocity(simulation.getMechanismVelocityRPS().getRotations());
		simulation.setInputVoltage(motor.getSimState().getMotorVoltage());
		//System.out.println(logpath+": "+motor.getSimState().getMotorVoltage());
	}

	@Override
	public void setBrake(boolean brake) {
		MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
		motor.getConfigurator().refresh(motorOutputConfigs);
		motorOutputConfigs.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		motor.getConfigurator().apply(motorOutputConfigs);

	}

	public void setPower(double amount) {
		motor.set(amount);
	}

	public void driveToPosition(Rotation2d position) {
		PositionVoltage positionVoltage = new PositionVoltage(position.getRotations());
		motor.setControl(positionVoltage);
	}

	public Rotation2d getPosition() {
		return Rotation2d.fromRotations(StatusSignal.getLatencyCompensatedValueAsDouble(position, velocity));
	}

	public Rotation2d getVelocity() {
		return Rotation2d.fromRotations(velocity.getValueAsDouble());
	}

	public double getVoltage() {
		return voltage.getValueAsDouble();
	}

	public double getCurrent() {
		return current.getValueAsDouble();
	}

	public void invertMotor() {
		MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
		motorOutputConfigs.Inverted=getMotorInvertedDirection();
		motor.getConfigurator().apply(motorOutputConfigs);
		direction = getMotorInvertedDirection();
	}

	public void setVoltage(double voltage) {
		motor.setVoltage(voltage);
	}

	public void setPosition(Rotation2d angle) {
		motor.setPosition(angle.getRotations());
	}

	private InvertedValue getMotorInvertedDirection() {
		return direction == InvertedValue.Clockwise_Positive ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
	}

	public void logAll() {
		updateInputs();
		Logger.recordOutput(logpath + "/position", position.getValue());
		Logger.recordOutput(logpath + "/velocity", velocity.getValue());
		Logger.recordOutput(logpath + "/voltage", voltage.getValue());
		Logger.recordOutput(logpath + "/current", current.getValue());
		Logger.recordOutput(logpath + "/isMotorConnected", isConnected());
	}

	public Rotation2d getPIDTarget() {
		return Rotation2d.fromRotations(motor.getClosedLoopReference().getValueAsDouble());
	}

	@Override
	public boolean isConnected() {
		return motor.isConnected();
	}

	@Override
	public void updateInputs(InputSignal<?>... inputSignals) {
		position.refresh();
		velocity.refresh();
		current.refresh();
		position.refresh();
	}

}
