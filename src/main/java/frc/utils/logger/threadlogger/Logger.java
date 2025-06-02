package frc.utils.logger.threadlogger;

import edu.wpi.first.util.WPISerializable;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

public final class Logger {

	private static LoggingThread logger;
	protected static final AtomicInteger failedLogsCount = new AtomicInteger();

	// separate locks allows to stop logging to a specific type (while writing on a different thread) of data without blocking other types
	// concurrent objects are not used here, since data loss is preferred over blocking the main thread
	protected static final ReadWriteLock integerLock = new ReentrantReadWriteLock(false);
	protected static final Map<String, Integer> integerData = new HashMap<>();
	protected static final ReadWriteLock booleanLock = new ReentrantReadWriteLock(false);
	protected static final Map<String, Boolean> booleanData = new HashMap<>();
	protected static final ReadWriteLock stringLock = new ReentrantReadWriteLock(false);
	protected static final Map<String, String> stringData = new HashMap<>();
	protected static final ReadWriteLock doubleLock = new ReentrantReadWriteLock(false);
	protected static final Map<String, Double> doubleData = new HashMap<>();
	protected static final ReadWriteLock doubleArrayLock = new ReentrantReadWriteLock(false);
	protected static final Map<String, double[]> doubleArrayData = new HashMap<>();
	protected static final ReadWriteLock wpilibObjectsLock = new ReentrantReadWriteLock(false);
	protected static final Map<String, WPISerializable> wpilibObjectsData = new HashMap<>();
	protected static final ReadWriteLock enumLock = new ReentrantReadWriteLock(false);
	protected static final Map<String, Enum> enumData = new HashMap<>();

	private static void startLogging() {
		logger = new LoggingThread("GBLogger");
		logger.setDaemon(true);
		logger.start();
	}

	private static void checkIfLoggerExistsOrElseStartLogger() {
		if (logger == null || !logger.isAlive()) {
			startLogging();
		}
	}

	public static <T> void logMap(Map<String, T> map, ReadWriteLock lock, String logPath, T data) {
		checkIfLoggerExistsOrElseStartLogger();
		lock.writeLock().lock();
		map.put(logPath, data);
		lock.writeLock().unlock();
	}

	public static void stopLogging() {
		if (logger != null && logger.isAlive()) {
			logger.interrupt();
		}
	}

	public static void recordOutput(String logPath, int data) {
		logMap(integerData, integerLock, logPath, data);
	}

	public static void recordOutput(String logPath, Boolean data) {
		logMap(booleanData, booleanLock, logPath, data);
	}

	public static void recordOutput(String logPath, String data) {
		logMap(stringData, stringLock, logPath, data);
	}

	public static void recordOutput(String logPath, double data) {
		logMap(doubleData, doubleLock, logPath, data);
	}

	public static void recordOutput(String logPath, double[] data) {
		logMap(doubleArrayData, doubleArrayLock, logPath, data);
	}

	public static void recordOutput(String logPath, WPISerializable data) {
		logMap(wpilibObjectsData, wpilibObjectsLock, logPath, data);
	}

	public static <T extends Enum<T>> void recordOutput(String logPath, T data) {
		logMap(enumData, enumLock, logPath, data);
	}

}
