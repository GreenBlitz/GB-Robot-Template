package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import frc.utils.time.TimeUtil;

import java.util.ArrayList;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReentrantLock;

public class OdometryThread<T> extends Thread {

	private ReentrantLock lock;
	private ArrayList<StatusSignal<T>> signals;
	private ArrayList<Queue<T>> signalsValuesQueues;
	private Queue<Double> timestamps;
	private double frequencyHertz;

	public OdometryThread(double frequencyHertz) {
		lock = new ReentrantLock();
		signals = new ArrayList<>();
		signalsValuesQueues = new ArrayList<>();
		timestamps = new ArrayBlockingQueue<>(50);
		this.frequencyHertz = frequencyHertz;

		setName("OdometryThread");
		setDaemon(true);
		start();
	}

	public Queue<T> addSignal(StatusSignal<T> signal) {
		Queue<T> queue = new ArrayBlockingQueue<>(50);

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

	private static void cleanQueue(Queue<?> queue) {
		for (int i = 0; i < queue.size() / 2; i++) {
			queue.poll();
		}
	}

	private void updateAllQueues(double timestamp) {
		for (int i = 0; i < signals.size(); i += 1) {
			Queue<T> queue = signalsValuesQueues.get(i);
			if (!queue.offer(signals.get(i).getValue())) {
				cleanQueue(queue);
				queue.offer(signals.get(i).getValue());
			}
		}
		timestamps.offer(timestamp);
	}

	private double calculateLatency() {
		double latency = 0.0;
		for (StatusSignal<T> signal : signals) {
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
