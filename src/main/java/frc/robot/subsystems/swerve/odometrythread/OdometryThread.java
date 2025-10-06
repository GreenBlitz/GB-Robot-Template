package frc.robot.subsystems.swerve.odometrythread;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.utils.ArrayUtil;
import frc.utils.Conversions;
import frc.utils.TimedValue;
import frc.utils.time.TimeUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReentrantLock;


public class OdometryThread extends Thread {

	public final ReentrantLock threadQueuesLock;

	private final ArrayList<Pair<StatusSignal<?>, StatusSignal<?>>> latencyAndSlopeSignals;
	private final ArrayList<Queue<TimedValue<Double>>> signalValuesQueues;
	private final ArrayList<Queue<TimedValue<Double>>> latencySignalValuesQueues;

	private final double frequencyHertz;
	private final int maxValueCapacityPerUpdate;
	private final boolean isBusChainCanFD;
	private final int threadPriority;
	private final double cycleSeconds;

	private StatusSignal<?>[] signals;
	private boolean isThreadPrioritySet;
	private double lastUpdateTimestamp;
	private double lastCycleLengthSeconds;

	public OdometryThread(double frequencyHertz, String name, int maxValueCapacityPerUpdate, boolean isBusChainCanFD, int threadPriority) {
		this.threadQueuesLock = new ReentrantLock();

		this.latencyAndSlopeSignals = new ArrayList<>();
		this.signalValuesQueues = new ArrayList<>();
		this.latencySignalValuesQueues = new ArrayList<>();

		this.frequencyHertz = frequencyHertz;
		this.maxValueCapacityPerUpdate = maxValueCapacityPerUpdate;
		this.isBusChainCanFD = isBusChainCanFD;
		this.threadPriority = threadPriority;
		this.cycleSeconds = Conversions.frequencyHertzToCycleTimeSeconds(frequencyHertz);

		this.signals = new StatusSignal[0];
		this.isThreadPrioritySet = false;
		this.lastUpdateTimestamp = 0;
		this.lastCycleLengthSeconds = 0;

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

	public double getLastCycleLengthSeconds() {
		return lastCycleLengthSeconds;
	}

	public Queue<TimedValue<Double>> addSignal(StatusSignal<?> signal) {
		Queue<TimedValue<Double>> queue = new ArrayBlockingQueue<>(maxValueCapacityPerUpdate);

		threadQueuesLock.lock();
		try {
			clearAllQueues();

			Phoenix6SignalBuilder.setFrequencyWithRetry(
				signal,
				Robot.ROBOT_TYPE.isSimulation() ? OdometryThreadConstants.SIMULATION_FREQUENCY_HERTZ : frequencyHertz
			);
			signals = ArrayUtil.addToFullArray(signal, signals, StatusSignal[]::new);
			signalValuesQueues.add(queue);
		} finally {
			threadQueuesLock.unlock();
		}
		update();
		return queue;
	}

	public void addLatencyAndSlopeSignals(
		StatusSignal<?> latencySignal,
		Queue<TimedValue<Double>> latencySignalQueue,
		StatusSignal<?> slopeSignal
	) {
		threadQueuesLock.lock();
		try {
			Pair<StatusSignal<?>, StatusSignal<?>> signals = new Pair<>(latencySignal, slopeSignal);
			latencyAndSlopeSignals.add(signals);
			latencySignalValuesQueues.add(latencySignalQueue);
		} finally {
			threadQueuesLock.unlock();
		}
	}

	private void clearAllQueues() {
		signalValuesQueues.forEach(java.util.Collection::clear);
	}

	private void updateAllQueues(double timestamp) {
		double latencyCompensatedTimestamp = timestamp;
		for (int i = 0; i < signals.length; i++) {
			Queue<TimedValue<Double>> queue = signalValuesQueues.get(i);
			queue.offer(new TimedValue<>(signals[i].getValueAsDouble(), latencyCompensatedTimestamp - signals[i].getTimestamp().getLatency()));
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

	public void update() {
		if (!isThreadPrioritySet) {
			if (Threads.setCurrentThreadPriority(true, threadPriority)) {
				isThreadPrioritySet = true;
			}
		}

		double currentTime = TimeUtil.getCurrentTimeSeconds();
		lastCycleLengthSeconds = currentTime - lastUpdateTimestamp;
		lastUpdateTimestamp = currentTime;

		StatusCode statusCode;
		if (isBusChainCanFD) {
			statusCode = StatusSignal.waitForAll(cycleSeconds, signals);
		} else {
			Timer.delay(cycleSeconds);
			statusCode = StatusSignal.refreshAll(signals);
		}
		if (!statusCode.isOK()) {
			return;
		}

		threadQueuesLock.lock();
		try {
			updateAllQueues(TimeUtil.getCurrentTimeSeconds());
		} finally {
			threadQueuesLock.unlock();
		}
	}

	private static double calculateAverageLatency(StatusSignal<?>[] signals) {
		if (signals.length == 0) {
			return 0;
		}
		double latencySum = Arrays.stream(signals).mapToDouble(signal -> signal.getTimestamp().getLatency()).sum();
		return latencySum / signals.length;
	}

}
