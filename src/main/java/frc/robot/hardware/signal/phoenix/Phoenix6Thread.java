package frc.robot.hardware.signal.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
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
	private final ArrayList<BaseStatusSignal> signals;
	private final ArrayList<Queue<Double>> queues;

	private Phoenix6Thread() {
		this.timestamps = new ArrayBlockingQueue<>(UPDATES_PER_ROBORIO_CYCLE);
		this.signals = new ArrayList<>();
		this.queues = new ArrayList<>();

		setName(Phoenix6Thread.class.getSimpleName());
		setDaemon(true);
		start();
	}

	public Queue<Double> getTimestampsQueue() {
		return timestamps;
	}

	public Queue<Double> registerSignal(BaseStatusSignal signal) {
		Queue<Double> signalQueue = new ArrayBlockingQueue<>(UPDATES_PER_ROBORIO_CYCLE);
		LOCK.lock();
		queues.add(signalQueue);
		signals.add(signal);
		PhoenixProUtils.checkWithRetry(() -> signal.setUpdateFrequency(FREQUENCY), FREQUENCY_SET_RETRIES);
		LOCK.unlock();
		return signalQueue;
	}

	@Override
	public void run() {
		Timer.delay(STARTING_DELAY_SECONDS);
		while (true) {
			LOCK.lock();
			fetchSignals();
			updateQueues();
			LOCK.unlock();
		}
	}

	private void fetchSignals() {
		BaseStatusSignal.waitForAll(CycleTimeUtils.DEFAULT_CYCLE_TIME_SECONDS, signals.toArray(new BaseStatusSignal[0]));
	}

	private void updateQueues() {
		double latencyCompensatedTimestamp = (Logger.getRealTimestamp() / 1.0e6) - signals.get(0).getTimestamp().getLatency();
		timestamps.offer(latencyCompensatedTimestamp);
		for (int i = 0; i < signals.size(); i++) {
			queues.get(i).offer(signals.get(i).getValueAsDouble());
		}
	}

}
