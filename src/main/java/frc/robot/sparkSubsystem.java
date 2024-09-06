package frc.robot;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.utils.GBSubsystem;

public class sparkSubsystem extends GBSubsystem {

	private final CANSparkMax motor;

	public sparkSubsystem(String logPath, int id) {
		super(logPath);
		motor = new CANSparkMax(id, MotorType.kBrushless);
	}

	public void setBrake(boolean brake) {
		CANSparkBase.IdleMode x = brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast;
		motor.setIdleMode(x);
	}

	@Override
	protected void subsystemPeriodic() {}

}
