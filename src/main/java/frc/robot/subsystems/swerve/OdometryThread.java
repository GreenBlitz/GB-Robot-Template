package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.ReentrantLock;

public class OdometryThread extends Thread {

	private ReentrantLock lock;
	private ArrayList<StatusSignal<Rotation2d>> signals;
	private ArrayList<Queue<Rotation2d>> signalsValuesQueues;
	private Queue<Double> timestamps;

	public OdometryThread() {
		lock = new ReentrantLock();
		signals = new ArrayList<>();
		signalsValuesQueues = new ArrayList<>();
		timestamps = new ArrayBlockingQueue<>(50);

		setName("OdometryThread");
		setDaemon(true);
		start();
	}

	public Queue<Rotation2d> addSignal(StatusSignal<Rotation2d> signal) {
		Queue<Rotation2d> queue = new ArrayBlockingQueue<>(50);
		signals.add(signal);

		lock.lock();
		try {
			signalsValuesQueues.add(queue);
			return signalsValuesQueues.get(signals.indexOf(signal));
		} catch (Exception e) {
		} finally {
			lock.unlock();
		}
		return queue;
	}

    private static void cleanQueue(ArrayBlockingQueue<Rotation2d> queue, int neededRemainingCapacity) {
        if (queue.remainingCapacity() > neededRemainingCapacity) {
            queue
        }
    }

    private void updateSignalsAndTimestampsQueues(double timestamp) {
        for (int i = 0; i < signals.size(); i += 1) {
            signalsValuesQueues.get(i).offer(signals.get(i).getValue());
        }
        timestamps.offer(timestamp);
    }

    private void update() {

    }

}
