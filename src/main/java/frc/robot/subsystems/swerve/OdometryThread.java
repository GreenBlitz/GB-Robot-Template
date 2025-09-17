package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import frc.utils.OdometryUtil;
import frc.utils.TimedValue;
import frc.utils.time.TimeUtil;

import java.util.ArrayList;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReentrantLock;


public class OdometryThread extends Thread {

	public final ReentrantLock THREAD_QUEUES_LOCK = new ReentrantLock();
	private final ArrayList<Queue<TimedValue<Double>>> signalValuesQueues;
	private final ArrayList<Queue<TimedValue<Double>>> latencySignalValuesQueues;
	private final double frequencyHertz;
	private final int maxValueCapacityPerUpdate;
	private final boolean isBusChainCanFD;
	private final int threadPriority;
	private final double cycleSeconds;
	private boolean isThreadPrioritySet;
	private StatusSignal<?>[] signals;
	private Pair<StatusSignal<?>, StatusSignal<?>>[] latencyAndSlopeSignals;

	public OdometryThread(double frequencyHertz, String name, int maxValueCapacityPerUpdate, boolean isBusChainCanFD, int threadPriority) {
		this.signalValuesQueues = new ArrayList<>();
		this.latencySignalValuesQueues = new ArrayList<>();
		this.frequencyHertz = frequencyHertz;
		this.maxValueCapacityPerUpdate = maxValueCapacityPerUpdate;
		this.isBusChainCanFD = isBusChainCanFD;
		this.threadPriority = threadPriority;
		this.cycleSeconds = OdometryUtil.getThreadCycleSeconds(frequencyHertz);
		this.isThreadPrioritySet = false;
		this.signals = new StatusSignal[0];
		this.latencyAndSlopeSignals = new Pair[0];

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

		THREAD_QUEUES_LOCK.lock();
		try {
			signals = OdometryUtil.addSignalToArray(OdometryUtil.getSignalWithCorrectFrequency(signal, frequencyHertz), signals);
			signalValuesQueues.add(queue);
			update();
		} finally {
			THREAD_QUEUES_LOCK.unlock();
		}
		return queue;
	}

	public void addLatencyAndSlopeSignals(
		StatusSignal<?> latencySignal,
		Queue<TimedValue<Double>> latencySignalQueue,
		StatusSignal<?> slopeSignal
	) {
		THREAD_QUEUES_LOCK.lock();
		try {
			Pair<StatusSignal<?>, StatusSignal<?>> signals = new Pair<>(latencySignal, slopeSignal);
			latencyAndSlopeSignals = OdometryUtil.addSignalsToArray(signals, latencyAndSlopeSignals);

			latencySignalValuesQueues.add(latencySignalQueue);
		} finally {
			THREAD_QUEUES_LOCK.unlock();
		}
	}

	private void clearAllQueues() {
		signalValuesQueues.forEach(java.util.Collection::clear);
	}

	private void updateAllQueues(double timestamp) {
		double latencyCompensatedTimestamp = timestamp - OdometryUtil.calculateLatency(signals);
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
		if (!isThreadPrioritySet) {
			if (Threads.setCurrentThreadPriority(true, threadPriority)) {
				isThreadPrioritySet = true;
			}
		}

		StatusCode statusCode;
		if (isBusChainCanFD) {
			statusCode = StatusSignal.waitForAll(cycleSeconds, signals);
		} else {
			Timer.delay(cycleSeconds);
			statusCode = StatusSignal.refreshAll(signals);
		}
		if (statusCode != StatusCode.OK) {
			return;
		}

		THREAD_QUEUES_LOCK.lock();
		try {
			updateAllQueues(TimeUtil.getCurrentTimeSeconds());
		} finally {
			THREAD_QUEUES_LOCK.unlock();
		}
	}

}
