package frc.utils;

import com.ctre.phoenix6.StatusSignal;
import frc.robot.Robot;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.swerve.OdometryThreadConstants;

import java.util.Arrays;

public class OdometryThreadUtil {

	public static double calculateAverageLatency(StatusSignal<?>[] signals) {
		if (signals.length == 0) {
			return 0;
		}
		double latencySum = Arrays.stream(signals).mapToDouble(signal -> signal.getTimestamp().getLatency()).sum();
		return latencySum / signals.length;
	}

	public static StatusSignal<?>[] addSignalToArray(StatusSignal<?> signal, StatusSignal<?>[] signals) {
		StatusSignal<?>[] newSignals = new StatusSignal[signals.length + 1];
		System.arraycopy(signals, 0, newSignals, 0, signals.length);
		newSignals[signals.length] = signal;
		return newSignals;
	}

	public static StatusSignal<?> cloneSignalWithCorrectFrequency(StatusSignal<?> signal, double threadFrequencyHertz) {
		if (Robot.ROBOT_TYPE.isSimulation()) {
			threadFrequencyHertz = OdometryThreadConstants.SIMULATION_FREQUENCY_HERTZ;
		}
		StatusSignal<?> signalClone = signal.clone();
		Phoenix6SignalBuilder.setFrequencyWithRetry(signalClone, threadFrequencyHertz);
		return signalClone;
	}

}
