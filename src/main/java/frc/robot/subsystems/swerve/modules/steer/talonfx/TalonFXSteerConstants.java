package frc.robot.subsystems.swerve.modules.steer.talonfx;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.devicewrappers.TalonFXWrapper;

public class TalonFXSteerConstants {

	protected static final int APPLY_CONFIG_RETRIES = 10;
	protected static final int NO_ENCODER_ID = -1;
	public static final String LOG_PATH_ADDITION = "Steer/";

	private final TalonFXWrapper motor;
	private final TalonFXSteerSignals signals;
	private final boolean enableFOC;

	public TalonFXSteerConstants(String logPathPrefix, CTREDeviceID steerMotorID, boolean inverted, TalonFXConfiguration configuration, boolean enableFOC) {
		this(logPathPrefix, steerMotorID, inverted, NO_ENCODER_ID, configuration, enableFOC);
	}

	public TalonFXSteerConstants(
		String logPathPrefix,
		CTREDeviceID steerMotorID,
		boolean inverted,
		int encoderID,
		TalonFXConfiguration configuration,
		boolean enableFOC
	) {
		TalonFXSteerConfigObject steerConfigObject = new TalonFXSteerConfigObject(logPathPrefix + LOG_PATH_ADDITION, steerMotorID, inverted, encoderID, configuration);
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
