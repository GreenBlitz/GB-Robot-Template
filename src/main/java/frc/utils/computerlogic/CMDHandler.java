package frc.utils.computerlogic;

import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.nio.file.Path;

public class CMDHandler {

    public static final boolean TEST = true;

    public static final String PATH_TO_REPOSITIORY = Path.of("").toAbsolutePath().toString();
    public static final String PATH_TO_PYTHON_DIRECTORY = PATH_TO_REPOSITIORY + "/src/main/python";
    public static final String PATH_TO_JAVA_DIRECTORY = PATH_TO_REPOSITIORY + "/src/main/java";

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
