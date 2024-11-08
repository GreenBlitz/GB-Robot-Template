package frc.robot.hard.phoenix6.motor;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.hard.phoenix6.Phoenix6DeviceID;
import frc.robot.hard.phoenix6.Phoenix6Utils;

public class TalonFXWrapper extends TalonFX {

	private static final int DEFAULT_CONFIG_NUMBER_OF_TRIES = 1;

	public TalonFXWrapper(int deviceId) {
		this(new Phoenix6DeviceID(deviceId));
	}

	public TalonFXWrapper(Phoenix6DeviceID ctreDeviceID) {
		super(ctreDeviceID.ID(), ctreDeviceID.busChain().getChainName());
	}

	public StatusCode applyConfiguration(TalonFXConfiguration configuration, int numberOfTries) {
		return Phoenix6Utils.checkWithRetry(() -> getConfigurator().apply(configuration), numberOfTries);
	}

	public StatusCode applyConfiguration(TalonFXConfiguration configuration) {
		return applyConfiguration(configuration, DEFAULT_CONFIG_NUMBER_OF_TRIES);
	}

}
