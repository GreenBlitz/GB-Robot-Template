package frc.utils.applicationsutils;

import frc.robot.constants.DirectoryPathsConstants;

import java.io.File;
import java.io.FileWriter;

public class OutputFile {

    private String name;
    private File file;

    private static OutputFile OUTPUT_FILE_OUTPUT_FILE = new OutputFile("OutputFile");

    public OutputFile(String name) {
        this.name = name;
        this.file = DirectoryPathsConstants.OUTPUT_FILES_DIRECTORY_PATH.resolve(name).toFile();

        FileCreator.ensureFolderExistence(DirectoryPathsConstants.OUTPUT_FILES_DIRECTORY_PATH.toFile());
        if (!this.file.exists()) {
            FileCreator.createFile(this.file);
        }
    }

    public void write(String text) {
        try {
            FileWriter myWriter = new FileWriter(file, true);
            myWriter.write("\n" + text);
            myWriter.close();
        }
        catch (Exception exception) {
            try {
                writeToOwnOutputFile(name);
            }
            catch (Exception ignoredException) {
                exception.printStackTrace();
            }
        }
    }

    private static void writeToOwnOutputFile(String nameOfOutputFile) {
        try {
            FileWriter myWriter = new FileWriter(OUTPUT_FILE_OUTPUT_FILE.file, true);
            myWriter.write("\n Unable to write to output file: " + nameOfOutputFile);
            myWriter.close();
        } catch (Exception exception) {

        }

    }

    public void openFile() {
        FileCreator.openFile(file);
    }

}
