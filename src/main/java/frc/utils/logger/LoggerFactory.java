package frc.utils.logger;

import com.ctre.phoenix6.SignalLogger;
import frc.robot.Robot;
import frc.utils.alerts.Alert;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.nio.file.Path;

public class LoggerFactory {

	public static void initializeLogger() {
		switch (Robot.ROBOT_TYPE) {
			case REAL -> startRealLogger();
			case SIMULATION -> startSimulationLogger();
		}
	}

	private static void startRealLogger() {
		SignalLogger.enableAutoLogging(true); // must be true to BusStatus to work

		if (LogSavePath.USB.isWritable()) {
			startLoggerOnUSB();
		} else {
			startLoggerOnRoborio();
			reportNoUSBFound();
		}
	}

	private static void reportNoUSBFound() {
		Logger.recordOutput(Alert.ALERT_LOG_PATH + "/Didn't find USB");
	}

	private static void startSimulationLogger() {
		startLogger(LogSavePath.COMPUTER);
	}

	private static void startLoggerOnUSB() {
		startLogger(LogSavePath.USB);
	}

	private static void startLoggerOnRoborio() {
		startLogger(LogSavePath.ROBORIO);
	}

	private static void startLogger(LogSavePath logSavePath) {
		setLoggingPath(logSavePath.getSavePath());
		Logger.addDataReceiver(new NT4Publisher());
		Logger.start();
		Logger.recordOutput("Logged In", logSavePath);
	}

	private static void setLoggingPath(Path path) {
		String stringPath = path.toString();
		SignalLogger.setPath(stringPath);
		Logger.addDataReceiver(new WPILOGWriter(stringPath));
	}

}
