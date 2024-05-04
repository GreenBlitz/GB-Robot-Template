package frc.utils.loggerutils;

import com.ctre.phoenix6.SignalLogger;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.nio.file.Files;
import java.nio.file.Path;

public class LoggerUtils {

    public static void startRealLogger() {
        SignalLogger.enableAutoLogging(LoggerConstants.IS_CTRE_AUTO_LOGGING);

        Path usbPath = Path.of(LoggerConstants.USB_LOG_PATH);
        if (Files.exists(usbPath) && Files.isWritable(usbPath)) {
            startLoggerOnUSB();
        }
        else {
            startLoggerOnRoborio();
        }
    }

    public static void startLoggerOnUSB() {
        setLoggingPath(LoggerConstants.USB_LOG_PATH);
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        Logger.recordOutput("Logged In", "USB");
    }

    public static void startLoggerOnRoborio() {
        setLoggingPath(LoggerConstants.SAFE_ROBORIO_LOG_PATH);
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        Logger.recordOutput("Logged In", "ROBORIO");
    }

    public static void startSimulationLogger() {
        setLoggingPath(LoggerConstants.SIMULATION_LOG_PATH);
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        Logger.recordOutput("Logged In", "COMPUTER");
    }

    public static void startReplayLogger() {
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_simulation")));
        Logger.start();
    }

    private static void setLoggingPath(String path) {
        SignalLogger.setPath(path);
        Logger.addDataReceiver(new WPILOGWriter(path));
    }

}