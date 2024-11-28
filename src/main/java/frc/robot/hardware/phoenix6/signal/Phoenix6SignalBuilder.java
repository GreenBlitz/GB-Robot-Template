package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.StatusSignal;
import frc.utils.AngleUnit;
import frc.robot.hardware.phoenix6.Phoenix6Utils;

public class Phoenix6SignalBuilder {

	private static final int UPDATE_FREQUENCY_RETRIES = 5;

	private static void setFrequencyWithRetry(StatusSignal<?> signal, double frequency) {
		Phoenix6Utils.checkWithRetry(() -> signal.setUpdateFrequency(frequency), UPDATE_FREQUENCY_RETRIES);
	}

	private static StatusSignal<?> cloneWithFrequency(StatusSignal<?> signal, double frequency) {
		StatusSignal<?> signalClone = signal.clone();
		setFrequencyWithRetry(signalClone, frequency);
		return signalClone;
	}

	public static Phoenix6DoubleSignal generatePhoenix6Signal(StatusSignal<?> signal, double frequency) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequency);
		return new Phoenix6DoubleSignal(signalClone.getName(), signalClone);
	}

	public static Phoenix6AngleSignal generatePhoenix6Signal(StatusSignal<?> signal, double frequency, AngleUnit angleUnit) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequency);
		return new Phoenix6AngleSignal(signalClone.getName(), signalClone, angleUnit);
	}

	public static Phoenix6LatencySignal generatePhoenix6Signal(
		StatusSignal<?> signal,
		SignalGetter signalSlope,
		double frequency,
		AngleUnit angleUnit
	) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequency);
		setFrequencyWithRetry(signalSlope.getSignal(), frequency);
		return new Phoenix6LatencySignal(signalClone.getName(), signalClone, signalSlope.getSignal(), angleUnit);
	}

	/**
	 * Use only if not fetching the slope signal!!!
	 */
	public static Phoenix6BothLatencySignal generatePhoenix6Signal(
		StatusSignal<?> signal,
		StatusSignal<?> signalSlope,
		double frequency,
		AngleUnit angleUnit
	) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequency);
		StatusSignal<?> signalSlopeClone = cloneWithFrequency(signalSlope, frequency);
		return new Phoenix6BothLatencySignal(signalClone.getName(), signalClone, signalSlopeClone, angleUnit);
	}

	public interface SignalGetter {

		/**
		 * For using refresh all with more signals...
		 */
		StatusSignal<?> getSignal();

	}

}
