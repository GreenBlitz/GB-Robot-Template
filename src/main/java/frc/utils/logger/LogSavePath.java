package frc.utils.logger;

import java.nio.file.Files;
import java.nio.file.Path;

enum LogSavePath {

    USB(Path.of("/media/sda1")),
    ROBORIO(Path.of("/home/lvuser/logs")),
    COMPUTER(Path.of("simulationlogs"));

    private final Path savePath;

    LogSavePath(Path savePath) {
        this.savePath = savePath;
    }

    Path getSavePath() {
        return savePath;
    }

    boolean isWritable() {
        return Files.exists(savePath) && Files.isWritable(savePath);
    }

}
