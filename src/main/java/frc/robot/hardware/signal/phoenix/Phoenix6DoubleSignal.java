package frc.robot.hardware.signal.phoenix;

import com.ctre.phoenix6.StatusSignal;
import frc.robot.hardware.signal.DoubleSignal;
import frc.robot.hardware.signal.TimedValue;

public class Phoenix6DoubleSignal extends DoubleSignal implements Phoenix6SignalBuilder.SignalGetter {

	private final StatusSignal<Double> statusSignal;

	protected Phoenix6DoubleSignal(String name, StatusSignal<Double> statusSignal) {
		super(name);
		this.statusSignal = statusSignal;
	}

	@Override
	protected TimedValue<Double> getNewValue() {
		return new TimedValue<>(statusSignal.getValue(), statusSignal.getTimestamp().getTime());
	}

	@Override
	public StatusSignal<Double> getSignal() {
		// For using refresh all with more signals...
		return statusSignal;
	}

}
