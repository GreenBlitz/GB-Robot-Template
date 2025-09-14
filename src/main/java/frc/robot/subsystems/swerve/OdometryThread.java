package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.utils.TimedValue;
import frc.utils.time.TimeUtil;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReentrantLock;

public class OdometryThread extends Thread {

	public static final ReentrantLock THREAD_LOCK = new ReentrantLock();
	private final ArrayList<Queue<TimedValue<Double>>> signalValuesQueues;
	private final ArrayList<Queue<TimedValue<Double>>> latencySignalValuesQueues;
	private final double frequencyHertz;
	private final int maxValueCapacityPerUpdate;
	private final boolean isBusChainCanFD;
	private StatusSignal<?>[] signals;
	private Pair<StatusSignal<?>, StatusSignal<?>>[] latencyAndSlopeSignals;

	public OdometryThread(double frequencyHertz, String name, int maxValueCapacityPerUpdate, boolean isBusChainCanFD, int threadPriority) {
		this.signalValuesQueues = new ArrayList<>();
		this.latencySignalValuesQueues = new ArrayList<>();
		this.frequencyHertz = frequencyHertz;
		this.maxValueCapacityPerUpdate = maxValueCapacityPerUpdate;
		this.isBusChainCanFD = isBusChainCanFD;
		this.signals = new StatusSignal[0];
		this.latencyAndSlopeSignals = new Pair[0];

		Threads.setCurrentThreadPriority(true, threadPriority);
		setName(name);
		setDaemon(true);
		start();
	}

	@Override
	public void run() {
		while (true) {
			update();
		}
	}

	public Queue<TimedValue<Double>> addSignal(StatusSignal<?> signal) {
		clearAllQueues();

		Queue<TimedValue<Double>> queue = new ArrayBlockingQueue<>(maxValueCapacityPerUpdate);

		THREAD_LOCK.lock();
		try {
			signals = addSignalToArray(getSignalWithCorrectFrequency(signal, frequencyHertz), signals);
			signalValuesQueues.add(queue);

			update();
			return queue;
		} finally {
			THREAD_LOCK.unlock();
		}
	}

	public void addLatencyAndSlopeSignals(
		StatusSignal<?> latencySignal,
		Queue<TimedValue<Double>> latencySignalQueue,
		StatusSignal<?> slopeSignal
	) {
		THREAD_LOCK.lock();
		try {
			Pair<StatusSignal<?>, StatusSignal<?>> signals = new Pair<>(latencySignal, slopeSignal);
			latencyAndSlopeSignals = addSignalsToArray(signals, latencyAndSlopeSignals);

			latencySignalValuesQueues.add(latencySignalQueue);
		} finally {
			THREAD_LOCK.unlock();
		}
	}

	private double calculateLatency() {
		if (signals.length == 0) {
			return 0;
		}
		double latency = 0.0;
		for (StatusSignal<?> signal : signals) {
			latency += signal.getTimestamp().getLatency();
		}
		return latency / signals.length;
	}

	private void clearAllQueues() {
		signalValuesQueues.forEach(Collection::clear);
	}

	private void updateAllQueues(double timestamp) {
		double latencyCompensatedTimestamp = timestamp - calculateLatency();
		for (int i = 0; i < signals.length; i++) {
			Queue<TimedValue<Double>> queue = signalValuesQueues.get(i);
			queue.offer(new TimedValue<>(signals[i].getValueAsDouble(), latencyCompensatedTimestamp));
		}

		for (int i = 0; i < latencyAndSlopeSignals.length; i++) {
			Queue<TimedValue<Double>> queue = latencySignalValuesQueues.get(i);
			queue.poll();
			queue
				.offer(
					new TimedValue<>(
						StatusSignal
							.getLatencyCompensatedValueAsDouble(latencyAndSlopeSignals[i].getFirst(), latencyAndSlopeSignals[i].getSecond()),
						timestamp
					)
				);
		}
	}

	private void update() {
		StatusCode statusCode;
		if (isBusChainCanFD) {
			statusCode = StatusSignal.waitForAll(getThreadCycleSeconds(frequencyHertz), signals);
		} else {
			Timer.delay(getThreadCycleSeconds(frequencyHertz));
			statusCode = StatusSignal.refreshAll(signals);
		}
		if (statusCode != StatusCode.OK) {
			return;
		}

		THREAD_LOCK.lock();
		try {
			updateAllQueues(TimeUtil.getCurrentTimeSeconds());
		} finally {
			THREAD_LOCK.unlock();
		}
	}

	private static double getThreadCycleSeconds(double frequencyHertz) {
		return 1 / frequencyHertz;
	}

	private static StatusSignal<?>[] addSignalToArray(StatusSignal<?> signal, StatusSignal<?>[] signals) {
		StatusSignal<?>[] newSignals = new StatusSignal[signals.length + 1];
		System.arraycopy(signals, 0, newSignals, 0, signals.length);
		newSignals[signals.length] = signal;
		return newSignals;
	}

	private static Pair<StatusSignal<?>, StatusSignal<?>>[] addSignalsToArray(
		Pair<StatusSignal<?>, StatusSignal<?>> signals,
		Pair<StatusSignal<?>, StatusSignal<?>>[] signalsArray
	) {
		Pair<StatusSignal<?>, StatusSignal<?>>[] newSignals = new Pair[signalsArray.length + 1];
		System.arraycopy(signalsArray, 0, newSignals, 0, signalsArray.length);
		newSignals[signalsArray.length] = signals;
		return newSignals;
	}

	private static StatusSignal<?> getSignalWithCorrectFrequency(StatusSignal<?> signal, double threadFrequencyHertz) {
		if (Robot.ROBOT_TYPE.isSimulation()) {
			threadFrequencyHertz = RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ;
		}
		StatusSignal<?> signalClone = signal.clone();
		Phoenix6SignalBuilder.setFrequencyWithRetry(signalClone, threadFrequencyHertz);
		return signalClone;
	}

}
