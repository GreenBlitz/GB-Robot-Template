package frc.utils.loggerutils;

import java.nio.file.Files;
import java.nio.file.Path;

public enum LogSaveSpot {

    USB(LoggerConstants.USB_LOG_PATH),
    ROBORIO(LoggerConstants.ROBORIO_LOG_PATH),
    COMPUTER(LoggerConstants.SIMULATION_LOG_PATH);

    public final String savePath;

    LogSaveSpot(String savePath) {
        this.savePath = savePath;
    }

    public boolean isWritable() {
        Path path = Path.of(savePath);
        return Files.exists(path) && Files.isWritable(path);
    }
}
