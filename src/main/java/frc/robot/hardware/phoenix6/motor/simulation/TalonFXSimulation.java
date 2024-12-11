package frc.robot.hardware.phoenix6.motor.simulation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.robot.hardware.phoenix6.motor.TalonFXWrapper;
import frc.robot.hardware.mechanisms.MechanismSimulation;
import frc.utils.battery.BatteryUtils;

public class TalonFXSimulation {

	private final double ROTOR_TO_SENSOR_RATIO = 1.0;

	private final TalonFXSimState motorSimState;
	private final MechanismSimulation mechanismSimulation;

	public TalonFXSimulation(TalonFXWrapper talonFXWrapper, TalonFXConfiguration configuration, MechanismSimulation simulation) {
		this.mechanismSimulation = simulation;
		this.motorSimState = talonFXWrapper.getSimState();
		ensureMotorConfig(talonFXWrapper, configuration);

		motorSimState.setSupplyVoltage(BatteryUtils.DEFAULT_VOLTAGE);
	}

	public void ensureMotorConfig(TalonFXWrapper talonFXWrapper, TalonFXConfiguration configuration) {
		configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
		if (configuration.Feedback.RotorToSensorRatio != ROTOR_TO_SENSOR_RATIO) {
			configuration.Feedback.SensorToMechanismRatio *= configuration.Feedback.RotorToSensorRatio / ROTOR_TO_SENSOR_RATIO;
			configuration.Feedback.RotorToSensorRatio = ROTOR_TO_SENSOR_RATIO;
		}
		talonFXWrapper.applyConfiguration(configuration);
	}

	public void updateMotor() {
		mechanismSimulation.setInputVoltage(motorSimState.getMotorVoltage());
		mechanismSimulation.updateMotor();
		motorSimState.setRawRotorPosition(mechanismSimulation.getRotorPosition().getRotations());
		motorSimState.setRotorVelocity(mechanismSimulation.getRotorVelocityAnglesPerSecond().getRotations());
	}

}
