package frc.robot.hard.phoenix6.signal;

import com.ctre.phoenix6.StatusSignal;
import frc.robot.hard.signal.DoubleSignal;
import frc.robot.hard.signal.TimedValue;
import frc.utils.time.TimeUtils;

public class Phoenix6DoubleSignal extends DoubleSignal implements Phoenix6SignalBuilder.SignalGetter {

	private final StatusSignal<Double> statusSignal;

	protected Phoenix6DoubleSignal(String name, StatusSignal<Double> statusSignal) {
		super(name);
		this.statusSignal = statusSignal;
	}

	@Override
	protected TimedValue<Double> getNewValue() {
		return new TimedValue<>(statusSignal.getValue(), TimeUtils.getCurrentTimeSeconds() - statusSignal.getTimestamp().getLatency());
	}

	@Override
	public StatusSignal<Double> getSignal() {
		// For using refresh all with more signals...
		return statusSignal;
	}

}
