package frc.utils;

import frc.robot.constants.*;
import org.littletonrobotics.junction.*;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.*;

public class LoggerUtils {

    public static void startRealLogger() {
        try {
            LoggerUtils.startLoggerOnUSB();
        }
        catch (Exception exception) {
            Logger.end();
            LoggerUtils.startLoggerOnRoborio();
        }
    }

    public static void startLoggerOnUSB() {
        Logger.addDataReceiver(new WPILOGWriter(RobotConstants.USB_LOG_PATH));
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        Logger.recordOutput("Logged In", "USB");
    }

    public static void startLoggerOnRoborio() {
        Logger.addDataReceiver(new WPILOGWriter(RobotConstants.SAFE_ROBORIO_LOG_PATH));
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        Logger.recordOutput("Logged In", "ROBORIO");
    }

    public static void startReplayLogger() {
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_simulation")));
        Logger.start();
    }

    public static void startSimulationLogger() {
        Logger.addDataReceiver(new NT4Publisher());
        Logger.addDataReceiver(new WPILOGWriter(SimulationConstants.SIMULATION_LOG_PATH));
        Logger.start();
        Logger.recordOutput("Logged In", "COMPUTER");
    }
}
