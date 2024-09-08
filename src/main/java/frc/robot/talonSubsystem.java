package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.utils.GBSubsystem;
import frc.utils.devicewrappers.TalonFXWrapper;

public class talonSubsystem extends GBSubsystem {

	private final TalonFXWrapper motor;

	public talonSubsystem(String logPath, int id) {
		super(logPath);
		motor = new TalonFXWrapper(id);
	}

	public void setBrake(boolean brake) {
		NeutralModeValue x = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		motor.setNeutralMode(x);
	}

	@Override
	protected void subsystemPeriodic() {}

}
