package frc.utils.loggerutils;

import com.ctre.phoenix6.SignalLogger;
import frc.robot.constants.LogPathsConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.nio.file.Path;

public class LoggerUtils {

    public static void startRealLogger() {
        SignalLogger.enableAutoLogging(LoggerConstants.IS_CTRE_AUTO_LOGGING);

        if (LogSaveSpot.USB.isWritable()) {
            startLoggerOnUSB();
        }
        else {
            startLoggerOnRoborio();
            reportNoUSBFound();
        }
    }

    private static void reportNoUSBFound() {
        Logger.recordOutput(LogPathsConstants.ALERT_LOG_PATH + "/Didn't find USB");
    }

    private static void startLoggerOnUSB() {
        startNonReplayLogger(LogSaveSpot.USB);
    }

    private static void startLoggerOnRoborio() {
        startNonReplayLogger(LogSaveSpot.ROBORIO);
    }

    public static void startSimulationLogger() {
        startNonReplayLogger(LogSaveSpot.COMPUTER);
    }

    public static void startReplayLogger() {
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_simulation")));
        Logger.start();
    }

    private static void startNonReplayLogger(LogSaveSpot logSaveSpot) {
        setLoggingPath(logSaveSpot.savePath);
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        Logger.recordOutput("Logged In", logSaveSpot);
    }

    private static void setLoggingPath(Path path) {
        String stringPath = path.toString();
        SignalLogger.setPath(stringPath);
        Logger.addDataReceiver(new WPILOGWriter(stringPath));
    }

}
