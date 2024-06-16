package frc.utils.applicationsutils;

import java.io.IOException;
import java.nio.file.Path;

public class CMDHandler {

    public static final String REPOSITORY_PATH = Path.of("").toAbsolutePath().toString();
    public static final String PATH_TO_PYTHON_DIRECTORY = REPOSITORY_PATH + "/src/main/python";
    public static final String PATH_TO_JAVA_DIRECTORY = REPOSITORY_PATH + "/src/main/java";

    public static void runCMDCommand(String command) {
        Runtime runtime = Runtime.getRuntime();
        try {
            runtime.exec(new String[]{"cmd.exe", "/c", command});
        } catch (IOException exception) {
            System.out.println("Unable to execute: " + command);// can't be logged because on computer side
        }
    }
}
