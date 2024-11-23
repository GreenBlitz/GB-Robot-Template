package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.StatusSignal;
import frc.robot.hardware.signal.AngleSignal;
import frc.robot.hardware.signal.TimedValue;
import frc.utils.AngleUnit;
import frc.utils.time.TimeUtils;

public class Phoenix6AngleSignal extends AngleSignal implements Phoenix6SignalBuilder.SignalGetter {

	private final StatusSignal<?> statusSignal;

	protected Phoenix6AngleSignal(String name, StatusSignal<?> statusSignal, AngleUnit angleUnit) {
		super(name, angleUnit);
		this.statusSignal = statusSignal;
	}

	@Override
	protected TimedValue<Double> getNewValue() {
		return new TimedValue<>(statusSignal.getValueAsDouble(), TimeUtils.getCurrentTimeSeconds() - statusSignal.getTimestamp().getLatency());
	}

	@Override
	public StatusSignal<?> getSignal() {
		// For using refresh all with more signals...
		return statusSignal;
	}

}
