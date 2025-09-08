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

	public static final ReentrantLock LOCK = new ReentrantLock();
	private final ArrayList<StatusSignal<?>> signals;
	private final ArrayList<Queue<TimedValue<Double>>> signalsValuesQueues;
	private final double frequencyHertz;
	private final int maxValueCapacityPerUpdate;

	public OdometryThread(double frequencyHertz, String name, int maxValueCapacityPerUpdate) {
		this.signals = new ArrayList<>();
		this.signalsValuesQueues = new ArrayList<>();
		this.frequencyHertz = frequencyHertz;
		this.maxValueCapacityPerUpdate = maxValueCapacityPerUpdate;

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

	public Queue<TimedValue<Double>> addSignal(StatusSignal<?> signal) {
		Queue<TimedValue<Double>> queue = new ArrayBlockingQueue<>(maxValueCapacityPerUpdate);

		LOCK.lock();
		try {
			signals.add(signal);
			signalsValuesQueues.add(queue);
			return queue;
		} catch (Exception e) {
		} finally {
			LOCK.unlock();
		}
		return queue;
	}

	private void updateAllQueues(double timestamp) {
		for (int i = 0; i < signals.size(); i += 1) {
			Queue<TimedValue<Double>> queue = signalsValuesQueues.get(i);
			queue.offer(new TimedValue<>(signals.get(i).getValueAsDouble(), timestamp));
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
		if (StatusSignal.waitForAll(getThreadCycleSeconds(frequencyHertz), signals.toArray(new StatusSignal[signals.size()])) != StatusCode.OK) {
			return;
		}
		double signalsTimestamp = TimeUtil.getCurrentTimeSeconds() - calculateLatency();

		LOCK.lock();
		try {
			updateAllQueues(signalsTimestamp);
		} catch (Exception e) {
		} finally {
			LOCK.unlock();
		}
	}

}
