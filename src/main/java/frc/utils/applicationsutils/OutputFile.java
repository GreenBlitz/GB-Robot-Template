package frc.utils.applicationsutils;

import frc.robot.constants.DirectoryPathsConstants;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class OutputFile {

    private String name;
    private File file;

    private static OutputFile SELF_OUTPUT_FILE = new OutputFile("OutputFile");

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
            writeUsingWriter(file, text);
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

    private static void writeUsingWriter(File file, String text) throws IOException {
        FileWriter myWriter = new FileWriter(file, true);
        myWriter.write(text + "\n");
        myWriter.close();
    }

    private static void writeToOwnOutputFile(String nameOfOutputFile) {
        try {
            writeUsingWriter(SELF_OUTPUT_FILE.file, "Unable to write to output file: " + nameOfOutputFile);
        } catch (Exception exception) {

        }
    }

    public void openFile() {
        FileCreator.openFile(file);
    }

}
