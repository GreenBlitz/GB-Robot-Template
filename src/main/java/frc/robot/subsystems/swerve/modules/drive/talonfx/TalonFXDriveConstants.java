package frc.robot.subsystems.swerve.modules.drive.talonfx;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.TalonFXWrapper;
import frc.robot.subsystems.swerve.modules.drive.DriveConstants;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class TalonFXDriveConstants {

	static final int APPLY_CONFIG_RETRIES = 10;

	private final TalonFXWrapper motor;
	private final TalonFXDriveSignals signals;
	private final boolean enableFOC;
	private final SysIdCalibrator.SysIdConfigInfo sysIdConfig;

	public TalonFXDriveConstants(
		Phoenix6DeviceID motorID,
		boolean inverted,
		TalonFXConfiguration configuration,
		boolean enableFOC,
		SysIdRoutine.Config sysIdConfig,
		String logPathPrefix
	) {
		TalonFXDriveConfigObject talonFXDriveConfigObject = new TalonFXDriveConfigObject(
			motorID,
			inverted,
			configuration,
			logPathPrefix + DriveConstants.LOG_PATH_ADDITION
		);
		this.motor = talonFXDriveConfigObject.getMotor();
		this.signals = talonFXDriveConfigObject.getSignals();
		this.enableFOC = enableFOC;
		this.sysIdConfig = new SysIdCalibrator.SysIdConfigInfo(sysIdConfig, true);
	}

	protected TalonFXWrapper getMotor() {
		return motor;
	}

	protected TalonFXDriveSignals getSignals() {
		return signals;
	}

	protected boolean getEnableFOC() {
		return enableFOC;
	}

	protected SysIdCalibrator.SysIdConfigInfo getSysIdConfig() {
		return sysIdConfig;
	}

}
