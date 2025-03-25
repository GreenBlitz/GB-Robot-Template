package frc.robot.hardware.phoenix6.signal;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import frc.robot.hardware.phoenix6.BusChain;
import frc.utils.math.AngleUnit;
import frc.robot.hardware.phoenix6.Phoenix6Util;

public class Phoenix6SignalBuilder {

	private static final int UPDATE_FREQUENCY_RETRIES = 5;

	public static BaseStatusSignal[] rioSignals = new BaseStatusSignal[0];
	public static BaseStatusSignal[] chassisSignals = new BaseStatusSignal[0];
	public static BaseStatusSignal[] superstructureSignals = new BaseStatusSignal[0];

	public static void refreshAll() {
		BaseStatusSignal.refreshAll(rioSignals);
		if (chassisSignals.length > 0) {
			BaseStatusSignal.refreshAll(chassisSignals);
		}
		BaseStatusSignal.refreshAll(superstructureSignals);
	}

	public static BaseStatusSignal[] addSignalToArray(BaseStatusSignal[] signals, BaseStatusSignal signal) {
		BaseStatusSignal[] newSignals = new BaseStatusSignal[signals.length + 1];
		System.arraycopy(signals, 0, newSignals, 0, signals.length);
		newSignals[newSignals.length - 1] = signal;
		return newSignals;
	}

	private static void setFrequencyWithRetry(StatusSignal<?> signal, double frequency) {
		Phoenix6Util.checkStatusCodeWithRetry(() -> signal.setUpdateFrequency(frequency), UPDATE_FREQUENCY_RETRIES);
	}

	private static StatusSignal<?> cloneWithFrequency(StatusSignal<?> signal, double frequency, BusChain busChain) {
		StatusSignal<?> signalClone = signal.clone();
		switch (busChain) {
			case ROBORIO -> rioSignals = addSignalToArray(rioSignals, signalClone);
			case SWERVE_CANIVORE -> {
				if (busChain.getChainName().equals("rio")) {
					rioSignals = addSignalToArray(rioSignals, signalClone);
				} else {
					chassisSignals = addSignalToArray(chassisSignals, signalClone);
				}
			}
			case SUPERSTRUCTURE_CANIVORE -> superstructureSignals = addSignalToArray(superstructureSignals, signalClone);
		}

		setFrequencyWithRetry(signalClone, frequency);
		return signalClone;
	}

	public static Phoenix6DoubleSignal build(StatusSignal<?> signal, double frequency, BusChain busChain) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequency, busChain);
		return new Phoenix6DoubleSignal(signalClone.getName(), signalClone);
	}

	public static Phoenix6AngleSignal build(StatusSignal<?> signal, double frequency, AngleUnit angleUnit, BusChain busChain) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequency, busChain);
		return new Phoenix6AngleSignal(signalClone.getName(), signalClone, angleUnit);
	}

	public static Phoenix6LatencySignal build(
		StatusSignal<?> signal,
		SignalGetter signalSlope,
		double frequency,
		AngleUnit angleUnit,
		BusChain busChain
	) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequency, busChain);
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
		AngleUnit angleUnit,
		BusChain busChain
	) {
		StatusSignal<?> signalClone = cloneWithFrequency(signal, frequency, busChain);
		StatusSignal<?> signalSlopeClone = cloneWithFrequency(signalSlope, frequency, busChain);
		return new Phoenix6LatencyAndSlopeSignal(signalClone.getName(), signalClone, signalSlopeClone, angleUnit);
	}

}
