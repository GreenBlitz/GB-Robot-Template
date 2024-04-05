package frc.utils;

import com.ctre.phoenix6.SignalLogger;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.Phoenix6Constants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.SimulationConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class LoggerUtils {

    public static void startRealLogger(){
        SignalLogger.enableAutoLogging(Phoenix6Constants.IS_AUTO_LOGGING);
        try {
            LoggerUtils.startLoggerOnUSB();
        } catch (Exception exception) {
            Logger.end();
            LoggerUtils.startLoggerOnRoborio();
        }
    }

    public static void startLoggerOnUSB(){
        setLoggingPath(RobotConstants.USB_LOG_PATH);
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        Logger.recordOutput("Logged In", "USB");
    }

    public static void startLoggerOnRoborio(){
        setLoggingPath(RobotConstants.SAFE_ROBORIO_LOG_PATH);
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        Logger.recordOutput("Logged In", "ROBORIO");
    }

    public static void startSimulationLogger(){
        setLoggingPath(SimulationConstants.SIMULATION_LOG_PATH);
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
        Logger.recordOutput("Logged In", "COMPUTER");
    }

    public static void startReplayLogger(){
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_simulation")));
        Logger.start();
    }

    private static void setLoggingPath(String path){
        SignalLogger.setPath(path);
        Logger.addDataReceiver(new WPILOGWriter(path));
    }
}
