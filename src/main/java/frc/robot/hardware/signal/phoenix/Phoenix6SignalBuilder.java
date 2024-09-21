package frc.robot.hardware.signal.phoenix;

import com.ctre.phoenix6.StatusSignal;
import frc.utils.AngleUnit;
import frc.robot.hardware.phoenix6.PhoenixProUtils;

public class Phoenix6SignalBuilder {

	private static final int UPDATE_FREQUENCY_RETRIES = 5;

	private static void setFrequencyWithRetry(StatusSignal<Double> signal, double frequency) {
		PhoenixProUtils.checkWithRetry(() -> signal.setUpdateFrequency(frequency), UPDATE_FREQUENCY_RETRIES);
	}

	private static StatusSignal<Double> cloneWithFrequency(StatusSignal<Double> signal, double frequency) {
		StatusSignal<Double> signalClone = signal.clone();
		setFrequencyWithRetry(signalClone, frequency);
		return signalClone;
	}

	public static Phoenix6DoubleSignal generatePhoenix6Signal(StatusSignal<Double> signal, double frequency) {
		StatusSignal<Double> signalClone = cloneWithFrequency(signal, frequency);
		return new Phoenix6DoubleSignal(signalClone.getName(), signalClone);
	}

	public static Phoenix6AngleSignal generatePhoenix6Signal(StatusSignal<Double> signal, double frequency, AngleUnit angleUnit) {
		StatusSignal<Double> signalClone = cloneWithFrequency(signal, frequency);
		return new Phoenix6AngleSignal(signalClone.getName(), signalClone, angleUnit);
	}

	//@formatter:off
	public static Phoenix6LatencySignal generatePhoenix6Signal(
            StatusSignal<Double> signal,
            SignalGetter signalSlope,
            double frequency,
            AngleUnit angleUnit
    ) {
		StatusSignal<Double> signalClone = cloneWithFrequency(signal, frequency);
		setFrequencyWithRetry(signalSlope.getSignal(), frequency);
		return new Phoenix6LatencySignal(signalClone.getName(), signalClone, signalSlope.getSignal(), angleUnit);
	}
	//@formatter:on

	/**
	 * Use only if not fetching the slope signal!!!
	 */
	public static Phoenix6BothLatencySignal generatePhoenix6Signal(
		StatusSignal<Double> signal,
		StatusSignal<Double> signalSlope,
		double frequency,
		AngleUnit angleUnit
	) {
		StatusSignal<Double> signalClone = cloneWithFrequency(signal, frequency);
		StatusSignal<Double> signalSlopeClone = cloneWithFrequency(signalSlope, frequency);
		return new Phoenix6BothLatencySignal(signalClone.getName(), signalClone, signalSlopeClone, angleUnit);
	}

	public interface SignalGetter {

		StatusSignal<Double> getSignal();

	}

}
