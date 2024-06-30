package frc.utils.applicationsutils;

import java.awt.*;
import java.io.File;
import java.io.FileWriter;

public class FileCreator {

    public static final OutputFile FILE_CREATOR_OUTPUT_FILE = new OutputFile("FileCreator.txt");

    public static void createFile(File file) {
        try {
            file.createNewFile();
        }
        catch (Exception exception) {
            FILE_CREATOR_OUTPUT_FILE.write( "Exception while creating file " + file + "\n" + exception);
        }
    }

    public static void openFile(File file) {
        try {
            if (!Desktop.isDesktopSupported()) {
                FILE_CREATOR_OUTPUT_FILE.write("Desktop does not support this file: " + file);
            }
            else if (file.exists()) {
                Desktop.getDesktop().open(file);
            }
        }
        catch (Exception exception) {
            FILE_CREATOR_OUTPUT_FILE.write("Exception While Opening File " + file + "\n" + exception);
        }
    }

    public static void writeToTextFile(File file, String text) {
        try {
            FileWriter myWriter = new FileWriter(file, true);
            myWriter.write(text);
            myWriter.close();
        }
        catch (Exception exception) {
            FILE_CREATOR_OUTPUT_FILE.write("Exception while writing to text file " + file + "\n" + exception);
        }
    }

    public static void clearTextFile(File file) {
        try {
            FileWriter myWriter = new FileWriter(file);
            myWriter.flush();
            myWriter.close();
        }
        catch (Exception exception) {
            FILE_CREATOR_OUTPUT_FILE.write("Exception while clearing text file " + file + "\n" + exception);
        }
    }

    public static void ensureFolderExistence(File folder) {
        if (!folder.exists()) {
            folder.mkdir();
        }
    }

}
