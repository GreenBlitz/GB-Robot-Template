package frc.robot.hardware.motor.phoenix6;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.ctre.PhoenixProUtils;

public class TalonFXWrapper extends TalonFX {

	private static final int DEFAULT_CONFIG_NUMBER_OF_TRIES = 1;

	public TalonFXWrapper(int deviceId) {
		this(new CTREDeviceID(deviceId));
	}

	public TalonFXWrapper(CTREDeviceID ctreDeviceID) {
		super(ctreDeviceID.ID(), ctreDeviceID.busChain().getChainName());
	}

	public StatusCode applyConfiguration(TalonFXConfiguration configuration, int numberOfTries) {
		return PhoenixProUtils.checkWithRetry(() -> getConfigurator().apply(configuration), numberOfTries);
	}

	public StatusCode applyConfiguration(TalonFXConfiguration configuration) {
		return applyConfiguration(configuration, DEFAULT_CONFIG_NUMBER_OF_TRIES);
	}

}
