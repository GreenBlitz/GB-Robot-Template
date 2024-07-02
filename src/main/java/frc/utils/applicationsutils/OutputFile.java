package frc.utils.applicationsutils;

import frc.robot.constants.DirectoryPathsConstants;

import java.awt.*;
import java.io.File;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;

public class OutputFile {

    private static final File SELF_OUTPUT_FILE = new OutputFile("OutputFile").path.toFile();

    private final String name;
    private final Path path;

    public OutputFile(String name) {
        this.name = name;
        this.path = DirectoryPathsConstants.OUTPUT_FILES_DIRECTORY_PATH.resolve(name);

        ensureFolderExists();
        ensureFileExists();
        clear();
    }

    private void ensureFolderExists() {
        Path directoryPath = DirectoryPathsConstants.OUTPUT_FILES_DIRECTORY_PATH;
        if (!Files.exists(directoryPath)) {
            directoryPath.toFile().mkdir();
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
        }
        catch (Exception exception) {
            reportFileError( "Exception while creating file " + path + "\n" + exception);
        }
    }

    public void write(String text) {
        try {
            Files.writeString(path, text + "\n", StandardOpenOption.APPEND);
        }
        catch (Exception exception) {
            reportFileError("Unable to write to output file: " + name + "\n" + exception);
        }
    }

    public void open() {
        try {
            if (!Desktop.isDesktopSupported()) {
                reportFileError("Desktop is not supported on this platform");
            }
            else if (Files.exists(path)) {
                Desktop.getDesktop().open(path.toFile());
            }
        }
        catch (Exception exception) {
            reportFileError("Exception While Opening File " + path + "\n" + exception);
        }
    }

    public void clear() {
        try {
            Files.writeString(path,"");
        }
        catch (Exception exception) {
            reportFileError("Exception while clearing text file " + path + "\n" + exception);
        }
    }

    private void reportFileError(String error) {
        try {
            Files.writeString(SELF_OUTPUT_FILE.toPath(), error + "\n", StandardOpenOption.APPEND);
        } catch (Exception exception) {
            System.out.println("Unable to write to Self Output file: \n" + exception);
        }
    }

}
