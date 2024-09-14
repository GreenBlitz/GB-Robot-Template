package frc.robot.hardware.signal.phoenix;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.Timer;
import frc.utils.Conversions;
import frc.utils.ctre.PhoenixProUtils;

import java.util.ArrayList;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;

public class Phoenix6Thread extends Thread {

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
	private static final double WAIT_TIME_SECONDS = 0.01;
	private static final double STARTING_DELAY_SECONDS = 0.01;

	private final Queue<Double> timestamps = new ArrayBlockingQueue<>(UPDATES_PER_ROBORIO_CYCLE);
	private final ArrayList<BaseStatusSignal> signals = new ArrayList<>();
	private final ArrayList<Queue<Double>> queues = new ArrayList<>();

	private Phoenix6Thread() {
		setName(Phoenix6Thread.class.getSimpleName());
		start();
	}

	public Queue<Double> getTimestamps() {
		return timestamps;
	}

	public Queue<Double> registerSignal(BaseStatusSignal signal) {
		Queue<Double> signalQueue = new ArrayBlockingQueue<>(UPDATES_PER_ROBORIO_CYCLE);
		queues.add(signalQueue);
		signals.add(signal);
		PhoenixProUtils.checkWithRetry(() -> signal.setUpdateFrequency(FREQUENCY), FREQUENCY_SET_RETRIES);
		return signalQueue;
	}

	@Override
	public void run() {
		Timer.delay(STARTING_DELAY_SECONDS);
		while (true) {
			fetchSignals();
			updateQueues();
		}
	}

	void fetchSignals() {
		BaseStatusSignal.waitForAll(WAIT_TIME_SECONDS, signals.toArray(new BaseStatusSignal[0]));
	}

	void updateQueues() {
		BaseStatusSignal signal = signals.get(0);
		queues.get(0).offer(signal.getValueAsDouble());
		double timestamp = Conversions.microSecondsToSeconds(HALUtil.getFPGATime()) - signal.getTimestamp().getLatency();
		timestamps.offer(timestamp);
		for (int i = 1; i < signals.size(); i++) {
			queues.get(i).offer(signals.get(i).getValueAsDouble());
		}
	}

}
