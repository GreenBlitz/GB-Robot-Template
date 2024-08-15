package frc.robot.subsystems.swerve.modules.drive.talonfx;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.devicewrappers.TalonFXWrapper;

public class TalonFXDriveConstants {

	private final TalonFXWrapper motor;

	private final TalonFXDriveSignals signals;

	private final boolean enableFOC;

	private final SysIdRoutine.Config sysIdConfig;

	public TalonFXDriveConstants(
			CTREDeviceID motorID,
			boolean inverted,
			TalonFXConfiguration configuration,
			boolean enableFOC,
			SysIdRoutine.Config sysIdConfig
	) {
		TalonFXDriveConfigObject talonFXDriveConfigObject = new TalonFXDriveConfigObject(motorID, inverted, configuration);
		this.motor = talonFXDriveConfigObject.getMotor();
		this.signals = talonFXDriveConfigObject.getSignals();
		this.enableFOC = enableFOC;
		this.sysIdConfig = sysIdConfig;
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

	public SysIdRoutine.Config getSysIdConfig() {
		return sysIdConfig;
	}

}
