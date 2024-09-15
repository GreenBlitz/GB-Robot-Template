package frc.robot.hardware.signal.phoenix;

import com.ctre.phoenix6.StatusSignal;
import frc.utils.AngleUnit;

public class Phoenix6BothLatencySignal extends Phoenix6LatencySignal {

	protected Phoenix6BothLatencySignal(
		String name,
		StatusSignal<Double> signal,
		StatusSignal<Double> slope,
		AngleUnit angleUnit
	) {
		super(name, signal, slope, angleUnit);
	}

	public StatusSignal<Double> getSignalSlope() {
		// For using refresh all with more signals...
		return slopeSignal;
	}

}
