package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.StatusSignal;
import frc.robot.hardware.phoenix6.BusChain;
import frc.utils.AngleUnit;
import frc.robot.hardware.phoenix6.Phoenix6Util;

public class Phoenix6SignalBuilder {

	private static final int UPDATE_FREQUENCY_RETRIES = 5;

	private static void setFrequencyWithRetry(StatusSignal<?> signal, double frequency) {
		Phoenix6Util.checkStatusCodeWithRetry(() -> signal.setUpdateFrequency(frequency), UPDATE_FREQUENCY_RETRIES);
	}

	private static StatusSignal<?> cloneWithFrequency(StatusSignal<?> signal, double frequency, BusChain busChain) {
		StatusSignal<?> signalClone = signal.clone();
		setFrequencyWithRetry(signalClone, frequency);
		busChain.registerSignal(signalClone);
		return signalClone;
	}

	public static Phoenix6DoubleSignal buildDoubleSignal(StatusSignal<?> signal, double frequency, BusChain busChain) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequency, busChain);
		return new Phoenix6DoubleSignal(signalClone.getName(), signalClone);
	}

	public static Phoenix6AngleSignal buildAngleSignal(StatusSignal<?> signal, double frequency, AngleUnit angleUnit, BusChain busChain) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequency, busChain);
		return new Phoenix6AngleSignal(signalClone.getName(), signalClone, angleUnit);
	}

	public static Phoenix6LatencySignal buildLatencySignal(
		StatusSignal<?> signal,
		StatusSignal<?> signalSlope,
		double frequency,
		AngleUnit angleUnit,
		BusChain busChain
	) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequency, busChain);
		StatusSignal<?> signalSlopeClone = cloneWithFrequency(signalSlope, frequency, busChain);
		return new Phoenix6LatencySignal(signalClone.getName(), signalClone, signalSlopeClone, angleUnit);
	}

	public static Phoenix6LatencySignal buildLatencySignalWithoutRegisteringSlope(
		StatusSignal<?> signal,
		StatusSignal<?> signalSlope,
		double frequency,
		AngleUnit angleUnit,
		BusChain busChain
	) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequency, busChain);
		setFrequencyWithRetry(signalSlope,frequency);
		return new Phoenix6LatencySignal(signalClone.getName(), signalClone, signalSlope, angleUnit);
	}

}
