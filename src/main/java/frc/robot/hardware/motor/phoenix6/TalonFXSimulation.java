package frc.robot.hardware.motor.phoenix6;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

public class TalonFXSimulation {

	private final TalonFXSimState motorSimState;
	private MechanismSimulation mechanismSimulation = null;

	public TalonFXSimulation(TalonFXWrapper talonFXWrapper, MechanismSimulation mechanismSimulation) {
		ensureMotorConfig(talonFXWrapper);
		this.mechanismSimulation = mechanismSimulation;
		this.motorSimState = talonFXWrapper.getSimState();
		motorSimState.setSupplyVoltage(12);
	}

	public void ensureMotorConfig(TalonFXWrapper talonFXWrapper) {
		TalonFXConfiguration configuration = new TalonFXConfiguration();
		talonFXWrapper.getConfigurator().refresh(configuration);
		configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
		if (configuration.Feedback.RotorToSensorRatio != 1.0) {
			configuration.Feedback.SensorToMechanismRatio *= configuration.Feedback.RotorToSensorRatio;
			configuration.Feedback.RotorToSensorRatio = 1.0;
		}
		talonFXWrapper.applyConfiguration(configuration);
	}

	public void updateMotor() {
		if (mechanismSimulation == null)
			return;
		mechanismSimulation.setInputVoltage(motorSimState.getMotorVoltage());
		mechanismSimulation.updateMotor();
		motorSimState.setRawRotorPosition(mechanismSimulation.getRotorPositionRotations().getRotations());
		motorSimState.setRotorVelocity(mechanismSimulation.getRotorVelocityRotationsPerSecond().getRotations());
	}

}
