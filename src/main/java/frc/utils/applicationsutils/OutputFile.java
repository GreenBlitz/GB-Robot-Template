package frc.utils.applicationsutils;

import frc.robot.constants.DirectoryPathsConstants;

import java.awt.*;
import java.io.File;
import java.io.FileWriter;
import java.nio.file.Files;
import java.nio.file.StandardOpenOption;

public class OutputFile {

    private static final File SELF_OUTPUT_FILE = getSelfOutputFile();

    private final String name;
    private final File file;

    public OutputFile(String name) {
        this.name = name;
        this.file = DirectoryPathsConstants.OUTPUT_FILES_DIRECTORY_PATH.resolve(name).toFile();

        ensureFolderExists();
        ensureFileExists();
        clear();
    }

    private void ensureFolderExists() {
        File folder = DirectoryPathsConstants.OUTPUT_FILES_DIRECTORY_PATH.toFile();
        if (!folder.exists()) {
            folder.mkdir();
        }
    }

    private void ensureFileExists() {
        if (!file.exists()) {
            createFile();
        }
    }

    private void createFile() {
        try {
            file.createNewFile();
        }
        catch (Exception exception) {
            reportFileError( "Exception while creating file " + file + "\n" + exception);
        }
    }

    public void write(String text) {
        try {
            Files.writeString(file.toPath(), text + "\n", StandardOpenOption.APPEND);
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
            else if (file.exists()) {
                Desktop.getDesktop().open(file);
            }
        }
        catch (Exception exception) {
            reportFileError("Exception While Opening File " + file + "\n" + exception);
        }
    }

    public void clear() {
        try {
            FileWriter myWriter = new FileWriter(file);
            myWriter.flush();
            myWriter.close();
        }
        catch (Exception exception) {
            reportFileError("Exception while clearing text file " + file + "\n" + exception);
        }
    }

    private void reportFileError(String error) {
        try {
            Files.writeString(SELF_OUTPUT_FILE.toPath(), error + "\n", StandardOpenOption.APPEND);
        } catch (Exception exception) {
            System.out.println("Unable to write to Self Output file: \n" + exception);
        }
    }

    private static File getSelfOutputFile() {
        OutputFile selfOutputFile = new OutputFile("OutputFile");
        selfOutputFile.clear();
        return selfOutputFile.file;
    }

}
