package frc.utils.loggerutils;

import java.nio.file.Path;
import java.nio.file.Paths;

class LoggerConstants {

    protected static final Path USB_LOG_PATH = Path.of("/media/sda1");

    protected static final Path ROBORIO_LOG_PATH = Path.of("/home/lvuser/logs");

    protected static final Path SIMULATION_LOG_PATH = Paths.get(System.getProperty("user.home"), "Desktop", "SimulationLogs");

    protected static final boolean IS_CTRE_AUTO_LOGGING = true; // Must be true for CANStatus to work

}
