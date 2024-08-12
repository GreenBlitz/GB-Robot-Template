package frc.robot.subsystems.swerve.modules.steer.talonfx;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.subsystems.swerve.modules.steer.SteerConstants;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.devicewrappers.TalonFXWrapper;

public class TalonFXSteerConstants {

	static final int APPLY_CONFIG_RETRIES = 10;
	protected static final int NO_ENCODER_ID = -1;

	private final TalonFXWrapper motor;
	private final TalonFXSteerSignals signals;
	private final boolean enableFOC;

	public TalonFXSteerConstants(CTREDeviceID steerMotorID, boolean inverted, TalonFXConfiguration configuration, boolean enableFOC, String logPathPrefix) {
		this(steerMotorID, inverted, NO_ENCODER_ID, configuration, enableFOC, logPathPrefix);
	}

	public TalonFXSteerConstants(
		CTREDeviceID steerMotorID,
		boolean inverted,
		int encoderID,
		TalonFXConfiguration configuration,
		boolean enableFOC,
		String logPathPrefix
	) {
		TalonFXSteerConfigObject steerConfigObject = new TalonFXSteerConfigObject(steerMotorID, inverted, encoderID, configuration, logPathPrefix + SteerConstants.LOG_PATH_ADDITION);
		this.motor = steerConfigObject.getMotor();
		this.signals = steerConfigObject.getSignals();
		this.enableFOC = enableFOC;
	}


	protected TalonFXWrapper getMotor() {
		return motor;
	}

	protected TalonFXSteerSignals getSignals() {
		return signals;
	}

	protected boolean getEnableFOC() {
		return enableFOC;
	}

}
