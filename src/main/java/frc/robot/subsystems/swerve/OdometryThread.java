package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
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
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReentrantLock;

public class OdometryThread extends Thread {

	public static final ReentrantLock LOCK = new ReentrantLock();
	private final StatusSignal<?>[] signals;
	private final Pair<StatusSignal<?>, StatusSignal<?>>[] latencyAndSlopeSignals;
	private final ArrayList<Queue<TimedValue<Double>>> signalValuesQueues;
	private final ArrayList<Pair<Queue<TimedValue<Double>>, Queue<TimedValue<Double>>>> latencyAndSlopeSignalValuesQueues;
	private final double frequencyHertz;
	private final int maxValueCapacityPerUpdate;
	private final boolean isBusChainCanFD;

	public OdometryThread(double frequencyHertz, String name, int maxValueCapacityPerUpdate, boolean isBusChainCanFD, int threadPriority) {
		this.signals = new StatusSignal[0];
		this.latencyAndSlopeSignals = new Pair[0];
		this.signalValuesQueues = new ArrayList<>();
		this.latencyAndSlopeSignalValuesQueues = new ArrayList<>();
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
		while (true) {
			update();
		}
	}

	public Queue<TimedValue<Double>> addSignal(StatusSignal<?> signal) {
		Queue<TimedValue<Double>> queue = new ArrayBlockingQueue<>(maxValueCapacityPerUpdate);

		LOCK.lock();
		try {
			addSignalToArray(getSignalWithCorrectFrequency(signal, frequencyHertz), signals);
			signalValuesQueues.add(queue);
			return queue;
		} finally {
			LOCK.unlock();
		}
	}

	public Pair<Queue<TimedValue<Double>>, Queue<TimedValue<Double>>> addLatencyAndSlopeSignals(
		StatusSignal<?> latencySignal,
		StatusSignal<?> slopeSignal
	) {
		Pair<Queue<TimedValue<Double>>, Queue<TimedValue<Double>>> queues = new Pair<>(
			new ArrayBlockingQueue<>(maxValueCapacityPerUpdate),
			new ArrayBlockingQueue<>(maxValueCapacityPerUpdate)
		);

		LOCK.lock();
		try {
			Pair<StatusSignal<?>, StatusSignal<?>> correctFrequencySignals = new Pair<>(
				getSignalWithCorrectFrequency(latencySignal, frequencyHertz),
				getSignalWithCorrectFrequency(slopeSignal, frequencyHertz)
			);
			addSignalToArray(correctFrequencySignals.getFirst(), signals);
			addSignalToArray(correctFrequencySignals.getSecond(), signals);
			addSignalsToArray(correctFrequencySignals, latencyAndSlopeSignals);

			signalValuesQueues.add(queues.getFirst());
			signalValuesQueues.add(queues.getSecond());
			latencyAndSlopeSignalValuesQueues.add(queues);

			return queues;
		} finally {
			LOCK.unlock();
		}
	}

	private void updateAllQueues(double timestamp) {
		double latencyCompensatedTimestamp = timestamp - calculateLatency();
		for (int i = 0; i < signals.length; i++) {
			Queue<TimedValue<Double>> queue = signalValuesQueues.get(i);
			queue.offer(new TimedValue<>(signals[i].getValueAsDouble(), latencyCompensatedTimestamp));
		}

		for (int i = 0; i < latencyAndSlopeSignals.length; i++) {
			Pair<Queue<TimedValue<Double>>, Queue<TimedValue<Double>>> queues = latencyAndSlopeSignalValuesQueues.get(i);
			queues.getFirst()
				.offer(
					new TimedValue<>(
						BaseStatusSignal
							.getLatencyCompensatedValueAsDouble(latencyAndSlopeSignals[i].getFirst(), latencyAndSlopeSignals[i].getSecond()),
						timestamp
					)
				);
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

		double currentTimestamp = TimeUtil.getCurrentTimeSeconds();
		LOCK.lock();
		try {
			updateAllQueues(currentTimestamp);
		} finally {
			LOCK.unlock();
		}
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

	private static void addSignalsToArray(
		Pair<StatusSignal<?>, StatusSignal<?>> signals,
		Pair<StatusSignal<?>, StatusSignal<?>>[] signalsArray
	) {
		Pair<StatusSignal<?>, StatusSignal<?>>[] newSignals = new Pair[signalsArray.length + 1];
		for (int i = 0; i < signalsArray.length; i++) {
			newSignals[i] = signalsArray[i];
		}
		newSignals[signalsArray.length] = signals;
		signalsArray = newSignals;
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
