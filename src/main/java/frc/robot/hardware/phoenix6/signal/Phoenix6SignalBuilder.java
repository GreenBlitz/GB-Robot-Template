package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.StatusSignal;
import frc.utils.math.AngleUnit;
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

	public static Phoenix6DoubleSignal build(StatusSignal<?> signal, double frequency) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequency);
		return new Phoenix6DoubleSignal(signalClone.getName(), signalClone);
	}

	public static Phoenix6AngleSignal build(StatusSignal<?> signal, double frequency, AngleUnit angleUnit) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequency);
		return new Phoenix6AngleSignal(signalClone.getName(), signalClone, angleUnit);
	}

	public static Phoenix6LatencySignal build(StatusSignal<?> signal, SignalGetter signalSlope, double frequency, AngleUnit angleUnit) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequency);
		setFrequencyWithRetry(signalSlope.getSignal(), frequency);
		return new Phoenix6LatencySignal(signalClone.getName(), signalClone, signalSlope.getSignal(), angleUnit);
	}

	/**
	 * Use this function only if you are not fetching the slope signal. Ex: If you care only about position and not velocity do:
	 * Phoenix6BothLatencySignal position = Phoenix6SignalBuilder.generatePhoenix6Signal(motor.getPosition(), motor.getVelocity(),...);
	 */
	public static Phoenix6LatencyAndSlopeSignal build(
		StatusSignal<?> signal,
		StatusSignal<?> signalSlope,
		double frequency,
		AngleUnit angleUnit
	) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequency);
		StatusSignal<?> signalSlopeClone = cloneWithFrequency(signalSlope, frequency);
		return new Phoenix6LatencyAndSlopeSignal(signalClone.getName(), signalClone, signalSlopeClone, angleUnit);
	}

}
