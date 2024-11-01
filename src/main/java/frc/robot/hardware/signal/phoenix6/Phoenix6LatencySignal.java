package frc.robot.hardware.signal.phoenix6;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import frc.robot.hardware.signal.AngleSignal;
import frc.robot.hardware.signal.TimedValue;
import frc.utils.AngleUnit;
import frc.utils.time.TimeUtils;

public class Phoenix6LatencySignal extends AngleSignal implements Phoenix6SignalBuilder.SignalGetter {

	private final StatusSignal<Double> signal;
	protected final StatusSignal<Double> slopeSignal;

	protected Phoenix6LatencySignal(String name, StatusSignal<Double> signal, StatusSignal<Double> slopeSignal, AngleUnit angleUnit) {
		super(name, angleUnit);
		this.signal = signal;
		this.slopeSignal = slopeSignal;
	}

	@Override
	protected TimedValue<Double> getNewValue() {
		return new TimedValue<>(BaseStatusSignal.getLatencyCompensatedValue(signal, slopeSignal), TimeUtils.getCurrentTimeSeconds());
	}

	@Override
	public StatusSignal<Double> getSignal() {
		// For using refresh all with more signals...
		return signal;
	}

}
