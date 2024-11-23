package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.StatusSignal;
import frc.utils.AngleUnit;

public class Phoenix6BothLatencySignal extends Phoenix6LatencySignal {

	protected Phoenix6BothLatencySignal(String name, StatusSignal<?> signal, StatusSignal<?> slope, AngleUnit angleUnit) {
		super(name, signal, slope, angleUnit);
	}

	public StatusSignal<?> getSignalSlope() {
		// For using refresh all with more signals...
		return slopeSignal;
	}

}
