package frc.utils.logger.threadlogger;

import edu.wpi.first.util.WPISerializable;
import frc.utils.logger.ILogger;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.ReadWriteLock;

public final class LoggingThread extends Thread {


	public static int NORM_PRIORITY = 2;
	public static int MAX_PRIORITY = 4;

	private int failedLogsCount = 0;

	public LoggingThread(String name) {
		super(name);
	}

	public LoggingThread(String name, ThreadGroup threadGroup) {
		super(threadGroup, name);
	}

	// One time object creation for each type of logger, instead of recreating them every time log() is called. Improves performance.
	final ILogger<Integer> integerLogger = org.littletonrobotics.junction.Logger::recordOutput;
	final ILogger<Boolean> booleanLogger = org.littletonrobotics.junction.Logger::recordOutput;
	final ILogger<String> stringLogger = org.littletonrobotics.junction.Logger::recordOutput;
	final ILogger<Double> doubleLogger = org.littletonrobotics.junction.Logger::recordOutput;
	final ILogger<double[]> doubleArrayLogger = org.littletonrobotics.junction.Logger::recordOutput;
	final ILogger<WPISerializable> wpilibObjectsLogger = org.littletonrobotics.junction.Logger::recordOutput;
	final ILogger<Enum> enumLogger = org.littletonrobotics.junction.Logger::recordOutput;

	private void log() {
		org.littletonrobotics.junction.Logger.recordOutput(LoggerConstants.ROOT_LOG_PATH + "FailedLogsCount", failedLogsCount);
		log(Logger.integerData, Logger.integerLock, integerLogger);
		log(Logger.booleanData, Logger.booleanLock, booleanLogger);
		log(Logger.stringData, Logger.stringLock, stringLogger);
		log(Logger.doubleData, Logger.doubleLock, doubleLogger);
		log(Logger.doubleArrayData, Logger.doubleArrayLock, doubleArrayLogger);
		log(Logger.wpilibObjectsData, Logger.wpilibObjectsLock, wpilibObjectsLogger);
		log(Logger.enumData, Logger.enumLock, enumLogger);
	}

	private <T> void log(Map<String, T> map, ReadWriteLock lock, ILogger<? super T> logger) {
		if (lock.readLock().tryLock()) {
			List<T> dataInMainThread = new ArrayList<>(map.size());
			List<String> logPathsInMainThread;
			try {
				logPathsInMainThread = map.keySet().stream().toList();
				for (String logPath : logPathsInMainThread) {
					dataInMainThread.add(map.get(logPath));
				}
			} finally {
				lock.readLock().unlock();
			}
			// actual logging happens outside the lock
			for (int i = 0; i < dataInMainThread.size(); i++) {
				logger.log(LoggerConstants.ROOT_LOG_PATH + logPathsInMainThread.get(i), dataInMainThread.get(i));
			}
		} else {
			// If the lock is not available, we skip logging this time.
			// This prevents blocking the main thread if the lock is held by another thread.
			failedLogsCount++;
		}
	}

	@Override
	public void run() {
		super.run();
		while (this.isAlive() && !Thread.currentThread().isInterrupted()) {
			this.log();
		}
	}

	@Override
	public void interrupt() {
		super.interrupt();
		// ensure any resources the thread might have created are cleaned up
		// right now there are no resources to clean up, but I left this here for future-proofing
	}

}
