package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.signal.AngleSignal;
import frc.utils.math.AngleUnit;
import frc.utils.TimedValue;
import frc.utils.time.TimeUtil;

public class Phoenix6AngleSignal extends AngleSignal implements SignalGetter {

	private final StatusSignal<?> statusSignal;

	protected Phoenix6AngleSignal(String name, StatusSignal<?> statusSignal, AngleUnit angleUnit) {
		super(name, angleUnit);
		this.statusSignal = statusSignal;
	}

	/**
	 * For using refresh all with more signals...
	 */
	@Override
	public StatusSignal<?> getSignal() {
		return statusSignal;
	}

	@Override
	protected void updateValue(TimedValue<Rotation2d> timedValue) {
		timedValue.setValue(angleUnit.toRotation2d(statusSignal.getValueAsDouble()));
		timedValue.setTimestamp(TimeUtil.getCurrentTimeSeconds() - statusSignal.getTimestamp().getLatency());
	}

}
