package frc.utils.applicationsutils;

import frc.robot.constants.DirectoryPathsConstants;

import java.awt.*;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class OutputFile {

    private static final OutputFile SELF_OUTPUT_FILE = new OutputFile("OutputFile");

    private String name;
    private File file;

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
            SELF_OUTPUT_FILE.write( "Exception while creating file " + file + "\n" + exception);
        }
    }

    public void write(String text) {
        try {
            writeUsingWriter(file, text);
        }
        catch (Exception exception) {
            reportFileError(name);
        }
    }

    public void openFile() {
        try {
            if (!Desktop.isDesktopSupported()) {
                SELF_OUTPUT_FILE.write("Desktop does not support this file: " + file);
            }
            else if (file.exists()) {
                Desktop.getDesktop().open(file);
            }
        }
        catch (Exception exception) {
            SELF_OUTPUT_FILE.write("Exception While Opening File " + file + "\n" + exception);
        }
    }

    public void clear() {
        try {
            FileWriter myWriter = new FileWriter(file);
            myWriter.flush();
            myWriter.close();
        }
        catch (Exception exception) {
            SELF_OUTPUT_FILE.write("Exception while clearing text file " + file + "\n" + exception);
        }
    }

    private static void writeUsingWriter(File file, String text) throws IOException {
        FileWriter myWriter = new FileWriter(file, true);
        myWriter.write(text + "\n");
        myWriter.close();
    }

    private static void reportFileError(String nameOfOutputFile) {
        try {
            writeUsingWriter(SELF_OUTPUT_FILE.file, "Unable to write to output file: " + nameOfOutputFile);
        } catch (Exception exception) {
            System.out.println("Unable to write to Self Output file: \n" + exception);
        }
    }

}
