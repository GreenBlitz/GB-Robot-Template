package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.StatusSignal;

@FunctionalInterface
public interface SignalGetter {

	/**
	 * For using refresh all with more signals...
	 */
	StatusSignal<?> getSignal();

}
