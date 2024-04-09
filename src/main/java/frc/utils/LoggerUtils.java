package frc.utils;

import frc.autogenerate.BuildConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.SimulationConstants;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class LoggerUtils {

    public static void startRealLogger() {
        try {
            LoggerUtils.startLoggerOnUSB();
        } catch (Exception exception) {
            Logger.end();
            LoggerUtils.startLoggerOnRoborio();
        }
        logGitInformation();
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
        logGitInformation();
    }

    public static void logGitInformation() {
        final String gitSituation =
                switch (BuildConstants.DIRTY) {
                    case 0 -> "All changes committed";
                    case 1 -> "Uncomitted changes";
                    default -> "Unknown";
                };
        Logger.recordOutput("Git/ProjectName", BuildConstants.MAVEN_NAME); // todo - metadata
        Logger.recordOutput("Git/GitBranch", BuildConstants.GIT_BRANCH); // todo - metadata
        Logger.recordOutput("Git/GitDate", BuildConstants.GIT_DATE); // todo - metadata
        Logger.recordOutput("Git/BuildDate", BuildConstants.BUILD_DATE); // todo - metadata
        Logger.recordOutput("Git/GitDirty", gitSituation); // todo - metadata
    }
}
