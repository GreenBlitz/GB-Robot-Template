// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.utils.ctre;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.jni.CANBusJNI;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.poseestimation.PoseEstimatorConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.utils.cycletime.CycleTimeUtils;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;


/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When using a CANivore, the thread uses the "waitForAll"
 * blocking method to enable more consistent sampling. This also allows Phoenix Pro users to benefit from lower latency between devices using
 * CANivore time synchronization.
 */
public class Phoenix6SignalsThread extends Thread {


	public static final Lock SIGNALS_LOCK = new ReentrantLock();

	private static final int MAX_UPDATES_PER_RIO_CYCLE = 100;
	private static final double DELAY_STARTING_TIME_SECONDS = 5;

	private final List<Queue<Double>> queues = new ArrayList<>();
	private final Queue<Double> timestamps = new ArrayBlockingQueue<>(MAX_UPDATES_PER_RIO_CYCLE);
	private final ArrayList<StatusSignal<Double>> signals = new ArrayList<>();
	private final ArrayList<Boolean> isLatencySignals = new ArrayList<>();
	private boolean isCANFD = false; // Assuming that all the devices using in odometry have same can network.


	private static Phoenix6SignalsThread INSTANCE = null;

	public static Phoenix6SignalsThread getInstance() {
		if (INSTANCE == null) {
			INSTANCE = new Phoenix6SignalsThread();
		}
		return INSTANCE;
	}

	private Phoenix6SignalsThread() {
		setName("Phoenix6SignalsThread");
		setDaemon(true);
		start();
	}


	public Queue<Double> getTimestampQueue() {
		return timestamps;
	}


	public Queue<Double> registerLatencySignal(ParentDevice device, StatusSignal<Double> signal, StatusSignal<Double> signalSlope) {
		return registerSignals(true, device, new StatusSignal[] {signal, signalSlope});
	}

	public Queue<Double> registerRegularSignal(ParentDevice device, StatusSignal<Double> signal) {
		return registerSignals(false, device, new StatusSignal[] {signal});
	}

	private Queue<Double> registerSignals(boolean isLatencySignal, ParentDevice device, StatusSignal<Double>[] signals) {
		Queue<Double> queue = new ArrayBlockingQueue<>(MAX_UPDATES_PER_RIO_CYCLE);
		SIGNALS_LOCK.lock();
		Swerve.ODOMETRY_LOCK.lock();
		try {
			updateIsCANFD(device);
			registerSignals(signals);
			updateIsLatencySignals(isLatencySignal);
			queues.add(queue);
		} finally {
			SIGNALS_LOCK.unlock();
			Swerve.ODOMETRY_LOCK.unlock();
		}
		return queue;
	}

	private void registerSignals(StatusSignal<Double>[] signals) {
		for (StatusSignal<Double> signal : signals) {
			registerSignal(signal);
		}
	}

	private void registerSignal(StatusSignal<Double> signal) {
		signals.add(signal);
	}

	private void updateIsCANFD(ParentDevice device) {
		isCANFD = CANBusJNI.JNI_IsNetworkFD(device.getNetwork());
	}

	private void updateIsLatencySignals(boolean isLatencySignal) {
		isLatencySignals.add(isLatencySignal);
	}


	@Override
	public void run() {
		Timer.delay(DELAY_STARTING_TIME_SECONDS);
		while (true) {
			waitForUpdatesFromSignals();
			saveNewData();
		}
	}


	private void waitForUpdatesFromSignals() {
		SIGNALS_LOCK.lock();
		try {
			waitForAllSignals();
		} catch (InterruptedException exception) {
			exception.printStackTrace();
		} finally {
			SIGNALS_LOCK.unlock();
		}
	}

	private void waitForAllSignals() throws InterruptedException {
		if (isCANFD) {
			waitForCanFDSignals();
		} else {
			waitForNonCanFDSignals();
		}
	}

	private void waitForCanFDSignals() {
		BaseStatusSignal.waitForAll(CycleTimeUtils.DEFAULT_CYCLE_TIME_SECONDS, signals.toArray(StatusSignal[]::new));
	}

	private void waitForNonCanFDSignals() throws InterruptedException {
		Thread.sleep((long) (1000.0 / PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ));
		if (!signals.isEmpty()) {
			BaseStatusSignal.refreshAll(signals.toArray(StatusSignal[]::new));
		}
	}


	private void saveNewData() {
		double fpgaTimestamp = Logger.getRealTimestamp() / 1.0e6;

		Swerve.ODOMETRY_LOCK.lock();
		try {
			saveNewDataToQueues();
			timestamps.offer(fpgaTimestamp);
		} finally {
			Swerve.ODOMETRY_LOCK.unlock();
		}
	}

	private void saveNewDataToQueues() {
		for (int signalIndex = 0, queueIndex = 0; queueIndex < queues.size(); signalIndex++, queueIndex++) {
			if (isLatencySignals.get(queueIndex)) {
				saveLatencyValue(signalIndex, queueIndex);
				signalIndex++;
			} else {
				saveRegularValue(signalIndex, queueIndex);
			}
		}
	}

	private void saveLatencyValue(int index, int queueIndex) {
		queues.get(queueIndex).offer(BaseStatusSignal.getLatencyCompensatedValue(signals.get(index), signals.get(index + 1)));
	}

	private void saveRegularValue(int index, int queueIndex) {
		queues.get(queueIndex).offer(signals.get(index).getValueAsDouble());
	}

}
