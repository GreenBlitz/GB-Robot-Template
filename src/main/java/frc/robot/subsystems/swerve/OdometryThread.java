package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import frc.utils.TimedValue;
import frc.utils.time.TimeUtil;

import java.util.ArrayList;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReentrantLock;

public class OdometryThread extends Thread {

	private final ReentrantLock lock;
	private final ArrayList<StatusSignal<?>> signals;
	private final ArrayList<Queue<TimedValue<Double>>> signalsValuesQueues;
	private final double frequencyHertz;

	public OdometryThread(double frequencyHertz) {
		lock = new ReentrantLock();
		signals = new ArrayList<>();
		signalsValuesQueues = new ArrayList<>();
		this.frequencyHertz = frequencyHertz;

		setName("OdometryThread");
		setDaemon(true);
		start();
	}

	private static void cleanHalfOfQueue(Queue<?> queue) {
		for (int i = 0; i < queue.size() / 2; i++) {
			queue.poll();
		}
	}

	public Queue<TimedValue<Double>> addSignal(StatusSignal<?> signal) {
		Queue<TimedValue<Double>> queue = new ArrayBlockingQueue<>(50);

		lock.lock();
		try {
			signals.add(signal);
			signalsValuesQueues.add(queue);
			return queue;
		} catch (Exception e) {
		} finally {
			lock.unlock();
		}
		return queue;
	}

	private void updateAllQueues(double timestamp) {
		for (int i = 0; i < signals.size(); i += 1) {
			Queue<TimedValue<Double>> queue = signalsValuesQueues.get(i);
			if (!queue.offer(new TimedValue<>(signals.get(i).getValueAsDouble(), timestamp))) {
				cleanHalfOfQueue(queue);
				queue.offer(new TimedValue<>(signals.get(i).getValueAsDouble(), timestamp));
			}
		}
	}

	private double calculateLatency() {
		double latency = 0.0;
		for (StatusSignal<?> signal : signals) {
			latency += signal.getTimestamp().getLatency();
		}
		return latency / signals.size();
	}

	private void update() {
		if (StatusSignal.waitForAll(frequencyHertz, signals.toArray(new StatusSignal[signals.size()])) != StatusCode.OK) {
			return;
		}
		final double signalsTimestamp = TimeUtil.getCurrentTimeSeconds() - calculateLatency();

		lock.lock();
		try {
			updateAllQueues(signalsTimestamp);
		} catch (Exception e) {
		} finally {
			lock.unlock();
		}
	}

}
