package frc.robot.hardware.motor.phoenix6;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.utils.ctre.CTREDeviceID;

public class TalonFXWrapper extends TalonFX {

	public TalonFXWrapper(int deviceId) {
		this(new CTREDeviceID(deviceId));
	}

	public TalonFXWrapper(CTREDeviceID ctreDeviceID) {
		super(ctreDeviceID.ID(), ctreDeviceID.busChain().getChainName());
	}

	public StatusCode applyConfiguration(TalonFXConfiguration configuration) {
		return super.getConfigurator().apply(configuration);
	}


}
