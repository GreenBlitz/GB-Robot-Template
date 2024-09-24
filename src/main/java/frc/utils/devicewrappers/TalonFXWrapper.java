package frc.utils.devicewrappers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;

public class TalonFXWrapper extends TalonFX {

	public TalonFXWrapper(int deviceId) {
		this(new Phoenix6DeviceID(deviceId));
	}

	public TalonFXWrapper(Phoenix6DeviceID phoenix6DeviceID) {
		super(phoenix6DeviceID.ID(), phoenix6DeviceID.busChain().getChainName());
	}

	public StatusCode applyConfiguration(TalonFXConfiguration configuration) {
		return super.getConfigurator().apply(configuration);
	}


	/**
	 * Performs latency compensation on signal using the signalSlope and signal's latency to determine the magnitude of compensation.
	 */
	private double getLatencyCompensatedValue(StatusSignal<Double> value, StatusSignal<Double> valueSlope) {
		return BaseStatusSignal.getLatencyCompensatedValue(value, valueSlope);
	}

	/**
	 * Performs latency compensation on velocity
	 */
	public double getLatencyCompensatedVelocity() {
		return getLatencyCompensatedValue(this.getVelocity(), this.getAcceleration());
	}

	/**
	 * Performs latency compensation on position
	 */
	public double getLatencyCompensatedPosition() {
		return getLatencyCompensatedValue(this.getPosition(), this.getVelocity());
	}

}
