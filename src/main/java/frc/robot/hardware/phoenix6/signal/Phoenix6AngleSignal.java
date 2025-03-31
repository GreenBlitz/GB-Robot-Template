package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.StatusSignal;
import frc.robot.hardware.signal.AngleSignal;
import frc.utils.TimedValue;
import frc.utils.AngleUnit;
import frc.utils.time.TimeUtil;

public class Phoenix6AngleSignal extends AngleSignal implements SignalGetter {

	private final StatusSignal<?> statusSignal;

	protected Phoenix6AngleSignal(String name, StatusSignal<?> statusSignal, AngleUnit angleUnit) {
		super(name, angleUnit);
		this.statusSignal = statusSignal;
	}

	@Override
	protected TimedValue<Double> getNewValue() {
		return new TimedValue<>(statusSignal.getValueAsDouble(), TimeUtil.getCurrentTimeSeconds() - statusSignal.getTimestamp().getLatency());
	}

	/**
	 * For using refresh all with more signals...
	 */
	@Override
	public StatusSignal<?> getSignal() {
		return statusSignal;
	}

}
