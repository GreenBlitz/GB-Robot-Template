package frc.robot.subsystems.swerve.modules.steer.talonfx;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.talonfx.TalonFXSignals;
import frc.robot.subsystems.swerve.modules.steer.SteerConstants;
import frc.utils.calibration.sysid.SysIdCalibrator;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.devicewrappers.TalonFXWrapper;

public class TalonFXSteerConstants {

	static final int APPLY_CONFIG_RETRIES = 10;
	static final int NO_ENCODER_ID = -1;

	private final TalonFXWrapper motor;
	private final TalonFXSignals signals;
	private final SysIdCalibrator.SysIdConfigInfo sysIdConfigInfo;

	public TalonFXSteerConstants(
		CTREDeviceID steerMotorID,
		boolean inverted,
		TalonFXConfiguration configuration,
		SysIdRoutine.Config sysIdConfig,
		String logPathPrefix
	) {
		this(steerMotorID, inverted, NO_ENCODER_ID, configuration, sysIdConfig, logPathPrefix);
	}

	public TalonFXSteerConstants(
		CTREDeviceID steerMotorID,
		boolean inverted,
		int encoderID,
		TalonFXConfiguration configuration,
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
		this.sysIdConfigInfo = new SysIdCalibrator.SysIdConfigInfo(sysIdConfig, true);
	}


	protected TalonFXWrapper getMotor() {
		return motor;
	}

	protected TalonFXSignals getSignals() {
		return signals;
	}

	protected SysIdCalibrator.SysIdConfigInfo getSysIdConfigInfo() {
		return sysIdConfigInfo;
	}

}
