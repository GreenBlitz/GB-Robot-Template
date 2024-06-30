package frc.utils.applicationsutils;

import frc.robot.constants.DirectoryPathsConstants;

import java.awt.*;
import java.io.File;
import java.io.FileWriter;

public class FileCreator {

    public static final File OUTPUT_FILE = DirectoryPathsConstants.OUTPUT_FILES_DIRECTORY_PATH.resolve("FileCreator.txt").toFile();

    public static void createFile(File file) {
        try {
            file.createNewFile();
        }
        catch (Exception exception) {
            writeToOutputFile(OUTPUT_FILE, "Exception while creating file " + file + "\n" + exception);
        }
    }

    public static void writeToTextFile(File file, String text) {
        try {
            write(file, text);
        }
        catch (Exception exception) {
            writeToOutputFile(OUTPUT_FILE, "Exception while writing to text file " + file + "\n" + exception);
        }
    }

    private static void write(File file, String text) throws Exception {
        FileWriter myWriter = new FileWriter(file, true);
        myWriter.write(text);
        myWriter.close();
    }

    public static void clearTextFile(File file) {
        try {
            FileWriter myWriter = new FileWriter(file);
            myWriter.flush();
            myWriter.close();
        }
        catch (Exception exception) {
            writeToOutputFile(OUTPUT_FILE, "Exception while clearing text file " + file + "\n" + exception);
        }
    }

    public static void openFile(File file) {
        try {
            if (!Desktop.isDesktopSupported()) {
                writeToOutputFile(OUTPUT_FILE, "Desktop does not support this file: " + file);
            }
            else if (file.exists()) {
                Desktop.getDesktop().open(file);
            }
        }
        catch (Exception exception) {
            writeToOutputFile(OUTPUT_FILE, "Exception While Opening File " + file + "\n" + exception);
        }
    }

    public static void ensureFolderExistence(File folder) {
        if (!folder.exists()) {
            folder.mkdir();
        }
    }

    public static void writeToOutputFile(File outputFile, String text) {
        ensureFolderExistence(DirectoryPathsConstants.OUTPUT_FILES_DIRECTORY_PATH.toFile());
        if (!outputFile.exists()) {
            FileCreator.createFile(outputFile);
        }

        try {
            write(outputFile, "\n" + text);
        }
        catch (Exception exception) {
            exception.printStackTrace();
        }
    }

}
