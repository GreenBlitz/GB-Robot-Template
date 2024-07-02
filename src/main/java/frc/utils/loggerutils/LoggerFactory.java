package frc.utils.loggerutils;

import com.ctre.phoenix6.SignalLogger;
import frc.robot.constants.LogPathsConstants;
import frc.robot.constants.RobotConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.nio.file.Path;

public class LoggerFactory {

    public static void initializeLogger() {
        switch (RobotConstants.ROBOT_TYPE) {
            case REAL -> startRealLogger();
            case SIMULATION -> startSimulationLogger();
            case REPLAY -> startReplayLogger();
        }
    }

    private static void startRealLogger() {
        SignalLogger.enableAutoLogging(true); // must be true to BusStatus to work

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

    private static void startSimulationLogger() {
        startNonReplayLogger(LogSaveSpot.COMPUTER);
    }

    private static void startLoggerOnUSB() {
        startNonReplayLogger(LogSaveSpot.USB);
    }

    private static void startLoggerOnRoborio() {
        startNonReplayLogger(LogSaveSpot.ROBORIO);
    }

    private static void startReplayLogger() {
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_simulation")));
        Logger.start();
    }

    private static void startNonReplayLogger(LogSaveSpot logSaveSpot) {
        setLoggingPath(logSaveSpot.getSavePath());
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
