package frc.utils.loggerutils;

import java.nio.file.Files;
import java.nio.file.Path;

enum LogSaveSpot {

    USB(Path.of("/media/sda1")),
    ROBORIO(Path.of("/home/lvuser/logs")),
    COMPUTER(Path.of("simulationlogs"));

    private final Path savePath;

    LogSaveSpot(Path savePath) {
        this.savePath = savePath;
    }

    Path getSavePath() {
        return savePath;
    }

    boolean isWritable() {
        return Files.exists(savePath) && Files.isWritable(savePath);
    }

}
