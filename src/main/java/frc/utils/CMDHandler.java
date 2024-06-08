package frc.utils;

import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.nio.file.Path;

public class CMDHandler {

    public static final String PATH_TO_PYTHON_DIRECTORY = getRepositoryPath() + "/src/main/python";
    public static final String PATH_TO_JAVA_DIRECTORY = getRepositoryPath() + "/src/main/java";


    public static String getRepositoryPath() {
        return Path.of("").toAbsolutePath().toString();
    }

    public static void runCMDCommand(String command) {
        Runtime runtime = Runtime.getRuntime();
        try {
            runtime.exec(new String[]{"cmd.exe", "/c", command});
        }
        catch (IOException exception) {
            Logger.recordOutput("CMD/Command Exception", exception.toString());
        }
    }

}
