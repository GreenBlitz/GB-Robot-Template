package frc.utils;

import frc.robot.constants.DirectoryPathsConstants;

import java.awt.*;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;

public class ComputerLogFile {

    private static final String TYPE_OF_FILE = ".txt";

    private static final Path LOG_FILES_ACTIVITY_FILE_PATH = new ComputerLogFile("ComputerLogFile").path;

    private final String name;
    private final Path path;

    public ComputerLogFile(String name) {
        this.name = name;
        this.path = DirectoryPathsConstants.COMPUTER_LOG_FILES_DIRECTORY_PATH.resolve(name + TYPE_OF_FILE);

        ensureFolderExists();
        ensureFileExists();
        clear();
        reportMessageToFile("Initialized Computer Log File: " + name);
    }

    private void ensureFolderExists() {
        try {
            Files.createDirectories(DirectoryPathsConstants.COMPUTER_LOG_FILES_DIRECTORY_PATH);
        } catch (Exception exception) {
            reportMessageToFile("Could not ensure computer log files folder exists: " + exception);
        }
    }

    private void ensureFileExists() {
        if (!Files.exists(path)) {
            createFile();
        }
    }

    private void createFile() {
        try {
            Files.createFile(path);
        } catch (Exception exception) {
            reportMessageToFile("Exception while creating file " + name + "\n" + exception);
        }
    }

    public void write(String text) {
        try {
            Files.writeString(path, text + "\n", StandardOpenOption.APPEND);
        } catch (Exception exception) {
            reportMessageToFile("Unable to write to computer log file: " + name + "\n" + exception);
        }
    }

    public void open() {
        try {
            if (!Desktop.isDesktopSupported()) {
                reportMessageToFile("Desktop is not supported on this platform");
            } else if (Files.exists(path)) {
                Desktop.getDesktop().open(path.toFile());
                reportMessageToFile("Opened file: " + name);
            }
        } catch (Exception exception) {
            reportMessageToFile("Exception While Opening File " + name + "\n" + exception);
        }
    }

    public void clear() {
        try {
            Files.writeString(path, "");
        } catch (Exception exception) {
            reportMessageToFile("Exception while clearing computer log file " + name + "\n" + exception);
        }
    }

    private void reportMessageToFile(String message) {
        try {
            Files.writeString(LOG_FILES_ACTIVITY_FILE_PATH, message + "\n", StandardOpenOption.APPEND);
        } catch (Exception exception) {
            System.out.println(message + "\n\nUnable to write to own computer logging file: \n" + exception);
        }
    }

}
