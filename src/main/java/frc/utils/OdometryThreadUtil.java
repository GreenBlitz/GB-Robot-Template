package frc.utils;

import com.ctre.phoenix6.StatusSignal;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;

public class OdometryThreadUtil {

	public static double calculateLatency(StatusSignal<?>[] signals) {
		if (signals.length == 0) {
			return 0;
		}
		double latency = 0.0;
		for (StatusSignal<?> signal : signals) {
			latency += signal.getTimestamp().getLatency();
		}
		return latency / signals.length;
	}

	public static StatusSignal<?>[] addSignalToArray(StatusSignal<?> signal, StatusSignal<?>[] signals) {
		StatusSignal<?>[] newSignals = new StatusSignal[signals.length + 1];
		System.arraycopy(signals, 0, newSignals, 0, signals.length);
		newSignals[signals.length] = signal;
		return newSignals;
	}

	public static StatusSignal<?> getSignalWithCorrectFrequency(StatusSignal<?> signal, double threadFrequencyHertz) {
		if (Robot.ROBOT_TYPE.isSimulation()) {
			threadFrequencyHertz = RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ;
		}
		StatusSignal<?> signalClone = signal.clone();
		Phoenix6SignalBuilder.setFrequencyWithRetry(signalClone, threadFrequencyHertz);
		return signalClone;
	}

}
