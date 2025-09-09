package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import frc.utils.TimedValue;
import frc.utils.time.TimeUtil;

import java.util.ArrayList;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReentrantLock;

public class OdometryThread extends Thread {

	public static final ReentrantLock LOCK = new ReentrantLock();
	private final StatusSignal<?>[] signals;
	private final ArrayList<Queue<TimedValue<Double>>> signalsValuesQueues;
	private final double frequencyHertz;
	private final int maxValueCapacityPerUpdate;
	private final boolean isBusChainCanFD;

	public OdometryThread(double frequencyHertz, String name, int maxValueCapacityPerUpdate, boolean isBusChainCanFD, int threadPriority) {
		this.signals = new StatusSignal[0];
		this.signalsValuesQueues = new ArrayList<>();
		this.frequencyHertz = frequencyHertz;
		this.maxValueCapacityPerUpdate = maxValueCapacityPerUpdate;
		this.isBusChainCanFD = isBusChainCanFD;
		Threads.setCurrentThreadPriority(true, threadPriority);

		setName(name);
		setDaemon(true);
		start();
	}

	@Override
	public void run() {
		update();
	}

	private static double getThreadCycleSeconds(double frequencyHertz) {
		return 1 / frequencyHertz;
	}

	private static void addSignalToArray(StatusSignal<?> signal, StatusSignal<?>[] signals) {
		StatusSignal<?>[] newSignals = new StatusSignal[signals.length + 1];
		for (int i = 0; i < signals.length; i++) {
			newSignals[i] = signals[i];
		}
		newSignals[signals.length] = signal;
		signals = newSignals;
	}

	public Queue<TimedValue<Double>> addSignal(StatusSignal<?> signal) {
		Queue<TimedValue<Double>> queue = new ArrayBlockingQueue<>(maxValueCapacityPerUpdate);

		LOCK.lock();
		try {
			addSignalToArray(signal, signals);
			signalsValuesQueues.add(queue);
			return queue;
		} finally {
			LOCK.unlock();
		}
	}

	public double getFrequencyHertz() {
		return frequencyHertz;
	}

	private void updateAllQueues(double timestamp) {
		for (int i = 0; i < signals.length; i += 1) {
			Queue<TimedValue<Double>> queue = signalsValuesQueues.get(i);
			queue.offer(new TimedValue<>(signals[i].getValueAsDouble(), timestamp));
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

		double signalsTimestamp = TimeUtil.getCurrentTimeSeconds() - calculateLatency();
		LOCK.lock();
		try {
			updateAllQueues(signalsTimestamp);
		} finally {
			LOCK.unlock();
		}
	}

}
