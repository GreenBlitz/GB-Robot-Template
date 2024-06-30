package frc.utils.applicationsutils;

import frc.robot.constants.DirectoryPathsConstants;

import java.awt.*;
import java.io.File;
import java.io.FileWriter;

public class FileCreator {

    public static final File OUTPUT_FILE = DirectoryPathsConstants.OUTPUT_FILES_DIRECTORY_PATH.resolve("FileCreator.txt").toFile();

    public static File createFile(File file) {
        try {
            file.createNewFile();
            return file;
        }
        catch (Exception exception) {
            writeToOutputFile(OUTPUT_FILE, "\nException while creating file " + file + "\n" + exception, true);
            return null;
        }
    }

    public static void writeToTextFile(File file, String text, boolean isAppending) {
        try {
            FileWriter myWriter = new FileWriter(file,isAppending);
            myWriter.write(text);
            myWriter.close();
        }
        catch (Exception exception) {
            writeToOutputFile(OUTPUT_FILE, "\nException while writing to text file " + file + "\n" + exception, true);
        }
    }

    private static void safeWriteToTextFile(File file, String text, boolean isAppending) {
        try {
            FileWriter myWriter = new FileWriter(file,isAppending);
            myWriter.write(text);
            myWriter.close();
        }
        catch (Exception exception) {
            System.out.println(exception);
        }
    }

    public static void clearTextFile(File file) {
        try {
            FileWriter myWriter = new FileWriter(file);
            myWriter.flush();
            myWriter.close();
        }
        catch (Exception exception) {
            writeToOutputFile(OUTPUT_FILE, "\nException while clearing text file " + file + "\n" + exception, true);
        }
    }

    public static void openFile(File file) {
        try {
            if (!Desktop.isDesktopSupported()) {
                writeToOutputFile(OUTPUT_FILE, "\nDesktop does not support this file: " + file, true);
            }
            else if (file.exists()) {
                    Desktop.getDesktop().open(file);
            }
        }
        catch (Exception exception) {
            exception.printStackTrace();
        }
    }

    public static void ensureFolderExistence(File folder) {
        if (!folder.exists()) {
            folder.mkdir();
        }
    }

    public static void writeToOutputFile(File outputFile, String text, boolean isAppending) {
        ensureFolderExistence(DirectoryPathsConstants.OUTPUT_FILES_DIRECTORY_PATH.toFile());
        if (!outputFile.exists()) {
            FileCreator.createFile(outputFile);
        }
        safeWriteToTextFile(outputFile, text, isAppending);
    }

}
