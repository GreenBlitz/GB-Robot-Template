package frc.utils;

import frc.robot.constants.DirectoryPathsConstants;

import java.awt.*;
import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;

public class OutputFile {

    private static final String TYPE_OF_FILE = ".txt";

    private static final File PRINTING_FILE = new OutputFile("OutputFile").path.toFile();

    private final String name;
    private final Path path;

    public OutputFile(String name) {
        this.name = name;
        this.path = DirectoryPathsConstants.OUTPUT_FILES_DIRECTORY_PATH.resolve(name + TYPE_OF_FILE);

        ensureFolderExists();
        ensureFileExists();
        clear();
        reportMessageToFile("Created Output File: " + name);
    }

    private void ensureFolderExists() {
        try {
            Files.createDirectories(DirectoryPathsConstants.OUTPUT_FILES_DIRECTORY_PATH);
        } catch (Exception exception) {
            reportMessageToFile("Could not ensure output files folder exists: " + exception);
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
            reportMessageToFile("Unable to write to output file: " + name + "\n" + exception);
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
            reportMessageToFile("Exception while clearing text file " + name + "\n" + exception);
        }
    }

    private void reportMessageToFile(String message) {
        try {
            Files.writeString(PRINTING_FILE.toPath(), message + "\n", StandardOpenOption.APPEND);
        } catch (Exception exception) {
            System.out.println(message + "\n\nUnable to write to Self Output file: \n" + exception);
        }
    }

}
