package frc.robot.subsystems.swerve.modules.steer.talonfx;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.subsystems.swerve.modules.steer.SteerConstants;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class TalonFXSteerConstants {

	static final int APPLY_CONFIG_RETRIES = 10;
	static final int NO_ENCODER_ID = -1;

	private final TalonFXWrapper motor;
	private final TalonFXSteerSignals signals;
	private final boolean enableFOC;
	private final SysIdCalibrator.SysIdConfigInfo sysIdConfigInfo;

	public TalonFXSteerConstants(
		Phoenix6DeviceID steerMotorID,
		boolean inverted,
		TalonFXConfiguration configuration,
		boolean enableFOC,
		SysIdRoutine.Config sysIdConfig,
		String logPathPrefix
	) {
		this(steerMotorID, inverted, NO_ENCODER_ID, configuration, enableFOC, sysIdConfig, logPathPrefix);
	}

	public TalonFXSteerConstants(
		Phoenix6DeviceID steerMotorID,
		boolean inverted,
		int encoderID,
		TalonFXConfiguration configuration,
		boolean enableFOC,
		SysIdRoutine.Config sysIdConfig,
		String logPathPrefix
	) {
		TalonFXSteerConfigObject steerConfigObject = new TalonFXSteerConfigObject(
			steerMotorID,
			inverted,
			encoderID,
			configuration,
			logPathPrefix + SteerConstants.LOG_PATH_ADDITION
		);
		this.motor = steerConfigObject.getMotor();
		this.signals = steerConfigObject.getSignals();
		this.enableFOC = enableFOC;
		this.sysIdConfigInfo = new SysIdCalibrator.SysIdConfigInfo(sysIdConfig, true);
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

	protected SysIdCalibrator.SysIdConfigInfo getSysIdConfigInfo() {
		return sysIdConfigInfo;
	}

}
