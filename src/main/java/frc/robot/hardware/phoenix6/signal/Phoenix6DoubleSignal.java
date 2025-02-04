package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.StatusSignal;
import frc.robot.hardware.signal.DoubleSignal;
import frc.utils.TimedValue;
import frc.utils.time.TimeUtil;

public class Phoenix6DoubleSignal extends DoubleSignal implements SignalGetter {

	private final StatusSignal<?> statusSignal;

	protected Phoenix6DoubleSignal(String name, StatusSignal<?> statusSignal) {
		super(name);
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
