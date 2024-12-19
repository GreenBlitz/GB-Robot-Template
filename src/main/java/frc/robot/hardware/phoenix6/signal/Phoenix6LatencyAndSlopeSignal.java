package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.StatusSignal;
import frc.utils.AngleUnit;

public class Phoenix6LatencyAndSlopeSignal extends Phoenix6LatencySignal {

	protected Phoenix6LatencyAndSlopeSignal(String name, StatusSignal<?> signal, StatusSignal<?> slope, AngleUnit angleUnit) {
		super(name, signal, slope, angleUnit);
	}

	/**
	 * For using refresh all with more signals...
	 */
	public StatusSignal<?> getSlopeSignal() {
		return slopeSignal;
	}

}
