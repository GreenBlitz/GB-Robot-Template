package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import frc.utils.Conversions;
import frc.utils.OdometryThreadUtil;
import frc.utils.TimedValue;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReentrantLock;


public class OdometryThread extends Thread {

	public final ReentrantLock ThreadQueuesLock;

	private final ArrayList<Pair<StatusSignal<?>, StatusSignal<?>>> latencyAndSlopeSignals;
	private final ArrayList<Queue<TimedValue<Double>>> signalValuesQueues;
	private final ArrayList<Queue<TimedValue<Double>>> latencySignalValuesQueues;

	private final double frequencyHertz;
	private final int maxValueCapacityPerUpdate;
	private final boolean isBusChainCanFD;
	private final int threadPriority;
	private final double cycleSeconds;
	private final String logPath;

	private StatusSignal<?>[] signals;
	private boolean isThreadPrioritySet;
	private double lastUpdateTimestamp;

	public OdometryThread(
		double frequencyHertz,
		String name,
		int maxValueCapacityPerUpdate,
		boolean isBusChainCanFD,
		int threadPriority,
		String logPath
	) {
		this.ThreadQueuesLock = new ReentrantLock();

		this.latencyAndSlopeSignals = new ArrayList<>();
		this.signalValuesQueues = new ArrayList<>();
		this.latencySignalValuesQueues = new ArrayList<>();

		this.frequencyHertz = frequencyHertz;
		this.maxValueCapacityPerUpdate = maxValueCapacityPerUpdate;
		this.isBusChainCanFD = isBusChainCanFD;
		this.threadPriority = threadPriority;
		this.cycleSeconds = Conversions.frequencyHertzToCycleSeconds(frequencyHertz);
		this.logPath = logPath;

		this.signals = new StatusSignal[0];
		this.isThreadPrioritySet = false;
		this.lastUpdateTimestamp = 0;

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

		ThreadQueuesLock.lock();
		try {
			signals = OdometryThreadUtil.addSignalToArray(OdometryThreadUtil.getSignalWithCorrectFrequency(signal, frequencyHertz), signals);
			signalValuesQueues.add(queue);
			update();
		} finally {
			ThreadQueuesLock.unlock();
		}
		return queue;
	}

	public void addLatencyAndSlopeSignals(
		StatusSignal<?> latencySignal,
		Queue<TimedValue<Double>> latencySignalQueue,
		StatusSignal<?> slopeSignal
	) {
		ThreadQueuesLock.lock();
		try {
			Pair<StatusSignal<?>, StatusSignal<?>> signals = new Pair<>(latencySignal, slopeSignal);
			latencyAndSlopeSignals.add(signals);

			latencySignalValuesQueues.add(latencySignalQueue);
		} finally {
			ThreadQueuesLock.unlock();
		}
	}

	private void clearAllQueues() {
		signalValuesQueues.forEach(java.util.Collection::clear);
	}

	private void updateAllQueues(double timestamp) {
		double latencyCompensatedTimestamp = timestamp - OdometryThreadUtil.calculateLatency(signals);
		for (int i = 0; i < signals.length; i++) {
			Queue<TimedValue<Double>> queue = signalValuesQueues.get(i);
			queue.offer(new TimedValue<>(signals[i].getValueAsDouble(), latencyCompensatedTimestamp));
		}

		for (int i = 0; i < latencyAndSlopeSignals.size(); i++) {
			Queue<TimedValue<Double>> queue = latencySignalValuesQueues.get(i);
			queue.poll();
			queue.offer(
				new TimedValue<>(
					StatusSignal
						.getLatencyCompensatedValueAsDouble(latencyAndSlopeSignals.get(i).getFirst(), latencyAndSlopeSignals.get(i).getSecond()),
					timestamp
				)
			);
		}
	}

	private void update() {
		double currentTime = TimeUtil.getCurrentTimeSeconds();
		Logger.recordOutput(logPath + "/CycleTimeSeconds", lastUpdateTimestamp - currentTime);
		lastUpdateTimestamp = currentTime;

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

		ThreadQueuesLock.lock();
		try {
			updateAllQueues(TimeUtil.getCurrentTimeSeconds());
		} finally {
			ThreadQueuesLock.unlock();
		}
	}

}
