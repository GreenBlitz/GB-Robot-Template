package frc.robot.hardware.signal.phoenix;

import com.ctre.phoenix6.StatusSignal;
import frc.robot.hardware.signal.AngleSignal;
import frc.robot.hardware.signal.TimedValue;
import frc.utils.AngleUnit;
import frc.utils.time.TimeUtils;

public class Phoenix6AngleSignal extends AngleSignal implements Phoenix6SignalBuilder.SignalGetter {

	private final StatusSignal<Double> statusSignal;

	protected Phoenix6AngleSignal(String name, StatusSignal<Double> statusSignal, AngleUnit angleUnit) {
		super(name, angleUnit);
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
