package frc.robot.subsystems.swerve.modules.drive.talonfx;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.talonfx.TalonFXSignals;
import frc.robot.subsystems.swerve.modules.drive.DriveConstants;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.devicewrappers.TalonFXWrapper;

public class TalonFXDriveConstants {

	static final int APPLY_CONFIG_RETRIES = 10;

	private final TalonFXWrapper motor;
	private final TalonFXSignals signals;
	private final SysIdRoutine.Config sysIdConfig;

	public TalonFXDriveConstants(
		CTREDeviceID motorID,
		boolean inverted,
		TalonFXConfiguration configuration,
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
		this.sysIdConfig = sysIdConfig;
	}

	protected TalonFXWrapper getMotor() {
		return motor;
	}

	protected TalonFXSignals getSignals() {
		return signals;
	}

	protected SysIdRoutine.Config getSysIdConfig() {
		return sysIdConfig;
	}

}
