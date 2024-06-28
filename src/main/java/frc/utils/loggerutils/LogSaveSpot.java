package frc.utils.loggerutils;

import java.nio.file.Files;
import java.nio.file.Path;

public enum LogSaveSpot {

    USB(LoggerConstants.USB_LOG_PATH),
    ROBORIO(LoggerConstants.ROBORIO_LOG_PATH),
    COMPUTER(LoggerConstants.SIMULATION_LOG_PATH);

    public final Path savePath;

    LogSaveSpot(Path savePath) {
        this.savePath = savePath;
    }

    public boolean isWritable() {
        return Files.exists(savePath) && Files.isWritable(savePath);
    }
}
