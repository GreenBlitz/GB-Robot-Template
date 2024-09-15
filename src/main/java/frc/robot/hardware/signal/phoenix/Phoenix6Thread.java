package frc.robot.hardware.signal.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj.Timer;
import frc.utils.ctre.PhoenixProUtils;
import frc.utils.cycletime.CycleTimeUtils;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReentrantLock;

public class Phoenix6Thread extends Thread {

	public static final ReentrantLock LOCK = new ReentrantLock();

	private static Phoenix6Thread instance;

	public static Phoenix6Thread getInstance() {
		if (instance == null) {
			instance = new Phoenix6Thread();
		}
		return instance;
	}

	private static final int UPDATES_PER_ROBORIO_CYCLE = 20;
	private static final int FREQUENCY = 250;
	private static final int FREQUENCY_SET_RETRIES = 5;
	private static final double STARTING_DELAY_SECONDS = 5;

	private final Queue<Double> timestamps;
	private final ArrayList<StatusSignal<Double>> signals;
	private final ArrayList<Boolean> isLatency;
	private final ArrayList<Queue<Double>> queues;

	private Phoenix6Thread() {
		this.timestamps = new ArrayBlockingQueue<>(UPDATES_PER_ROBORIO_CYCLE);
		this.signals = new ArrayList<>();
		this.isLatency = new ArrayList<>();
		this.queues = new ArrayList<>();

		setName(Phoenix6Thread.class.getSimpleName());
		setDaemon(true);
		start();
	}

	public Queue<Double> getTimestampsQueue() {
		return timestamps;
	}

	public Queue<Double> registerSignal(StatusSignal<Double> signal) {
		Queue<Double> signalQueue = new ArrayBlockingQueue<>(UPDATES_PER_ROBORIO_CYCLE);
		PhoenixProUtils.checkWithRetry(() -> signal.setUpdateFrequency(FREQUENCY), FREQUENCY_SET_RETRIES);
		LOCK.lock();
		{
			queues.add(signalQueue);
			signals.add(signal);
			isLatency.add(false);
		}
		LOCK.unlock();
		return signalQueue;
	}

	public Queue<Double> registerSignal(StatusSignal<Double> signal, StatusSignal<Double> signalSlope) {
		Queue<Double> signalQueue = new ArrayBlockingQueue<>(UPDATES_PER_ROBORIO_CYCLE);
		PhoenixProUtils.checkWithRetry(() -> signal.setUpdateFrequency(FREQUENCY), FREQUENCY_SET_RETRIES);
		PhoenixProUtils.checkWithRetry(() -> signalSlope.setUpdateFrequency(FREQUENCY), FREQUENCY_SET_RETRIES);
		LOCK.lock();
		{
			queues.add(signalQueue);
			signals.add(signal);
			signals.add(signalSlope);
			isLatency.add(true);
		}
		LOCK.unlock();
		return signalQueue;
	}

	@Override
	public void run() {
		Timer.delay(STARTING_DELAY_SECONDS);
		while (true) {
			LOCK.lock();
			{
				fetchSignals();
				updateQueues();
			}
			LOCK.unlock();
		}
	}

	private void fetchSignals() {
		BaseStatusSignal.waitForAll(CycleTimeUtils.DEFAULT_CYCLE_TIME_SECONDS, signals.toArray(new BaseStatusSignal[0]));
	}

	private void updateQueues() {
		double latencyCompensatedTimestamp = (Logger.getRealTimestamp() / 1.0e6) - signals.get(0).getTimestamp().getLatency();
		timestamps.offer(latencyCompensatedTimestamp);
		for (int signalsIndex = 0, queuesIndex = 0; signalsIndex < signals.size(); signalsIndex++, queuesIndex++) {
			if (isLatency.get(queuesIndex)) {
				double latencyCompensatedValue = BaseStatusSignal.getLatencyCompensatedValue(signals.get(signalsIndex), signals.get(signalsIndex + 1));
				queues.get(queuesIndex).offer(latencyCompensatedValue);
				signalsIndex++;
			} else {
				queues.get(queuesIndex).offer(signals.get(signalsIndex).getValueAsDouble());
			}
		}
	}

}
