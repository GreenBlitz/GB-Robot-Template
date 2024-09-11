package frc.robot.hardware.signal.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import frc.robot.hardware.signal.AngleSignal;
import frc.robot.hardware.signal.DoubleSignal;
import frc.utils.AngleUnit;
import frc.utils.ctre.PhoenixProUtils;

public class Phoenix6SignalBuilder {

	private static final int UPDATE_FREQUENCY_RETRIES = 5;

	public interface SignalGetter {

		StatusSignal<Double> getSignal();

	}

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

	public static class Phoenix6DoubleSignal extends DoubleSignal implements SignalGetter {

		private final StatusSignal<Double> statusSignal;

		private Phoenix6DoubleSignal(String name, StatusSignal<Double> statusSignal) {
			super(name);
			this.statusSignal = statusSignal;
		}

		@Override
		protected Double getNewValue() {
			return statusSignal.getValue();
		}

		@Override
		public StatusSignal<Double> getSignal() {
			// For using refresh all with more signals...
			return statusSignal;
		}

	}

	public static class Phoenix6AngleSignal extends AngleSignal implements SignalGetter {

		private final StatusSignal<Double> statusSignal;

		private Phoenix6AngleSignal(String name, StatusSignal<Double> statusSignal, AngleUnit angleUnit) {
			super(name, angleUnit);
			this.statusSignal = statusSignal;
		}

		@Override
		protected double getNewValue() {
			return statusSignal.getValue();
		}

		@Override
		public StatusSignal<Double> getSignal() {
			// For using refresh all with more signals...
			return statusSignal;
		}

	}

	public static class Phoenix6LatencySignal extends AngleSignal implements SignalGetter {

		private final StatusSignal<Double> signal;
		protected final StatusSignal<Double> slopeSignal;

		private Phoenix6LatencySignal(
			String name,
			StatusSignal<Double> signal,
			StatusSignal<Double> slopeSignal,
			AngleUnit angleUnit
		) {
			super(name, angleUnit);
			this.signal = signal;
			this.slopeSignal = slopeSignal;
		}

		@Override
		protected double getNewValue() {
			return BaseStatusSignal.getLatencyCompensatedValue(signal, slopeSignal);
		}

		@Override
		public StatusSignal<Double> getSignal() {
			// For using refresh all with more signals...
			return signal;
		}

	}

	public static class Phoenix6BothLatencySignal extends Phoenix6LatencySignal {

		private Phoenix6BothLatencySignal(
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

}
