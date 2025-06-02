package frc.utils.logger.threadlogger;

import edu.wpi.first.util.WPISerializable;
import frc.utils.logger.ILogger;

import java.util.Map;
import java.util.concurrent.locks.ReadWriteLock;

public final class LoggingThread extends Thread {


    public static int NORM_PRIORITY = 2;
    public static int MAX_PRIORITY = 4;

    public LoggingThread(String name) {
        super(name);
    }

    public LoggingThread(String name, ThreadGroup threadGroup) {
        super(threadGroup, name);
    }

    // One time object creation for each type of logger, instead of recreating them every time log() is called. Improves performance.
    ILogger<Integer> integerLogger = org.littletonrobotics.junction.Logger::recordOutput;
    ILogger<Double> doubleLogger = org.littletonrobotics.junction.Logger::recordOutput;
    ILogger<double[]> doubleArrayLogger = org.littletonrobotics.junction.Logger::recordOutput;
    ILogger<WPISerializable> wpilibObjectsLogger = org.littletonrobotics.junction.Logger::recordOutput;

    private void log() {
        org.littletonrobotics.junction.Logger.recordOutput(LoggerConstants.ROOT_LOG_PATH + "FailedLogsCount", Logger.failedLogsCount.get());
        log(Logger.integerData, Logger.integerLock, integerLogger);
        log(Logger.doubleData, Logger.doubleLock, doubleLogger);
        log(Logger.doubleArrayData, Logger.doubleArrayLock, doubleArrayLogger);
        log(Logger.wpilibObjectsData, Logger.wpilibObjectsLock, wpilibObjectsLogger);
    }

    private<T> void log(Map<String, T> map, ReadWriteLock lock, ILogger<? super T> logger) {
        if (lock.readLock().tryLock()) {
            try {
                for (String logPath : map.keySet()) {
                    logger.log(LoggerConstants.ROOT_LOG_PATH + logPath, map.get(logPath));
                }
            } finally {
                lock.readLock().unlock();
            }
        } else {
            // If the lock is not available, we skip logging this time.
            // This prevents blocking the main thread if the lock is held by another thread.
            Logger.failedLogsCount.incrementAndGet();
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
