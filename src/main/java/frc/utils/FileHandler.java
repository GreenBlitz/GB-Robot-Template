package frc.utils;

import java.io.IOException;
import java.nio.file.Path;

    public class FileHandler {

    public static String getPathToPythonDirectory() {
        return getRepositoryPath() + "/src/main/python";
    }

    public static String getRepositoryPath() {
        return Path.of("").toAbsolutePath().toString();
    }

    public static void runCmd(String command) {
        Runtime rt = Runtime.getRuntime();
        try {
            rt.exec(new String[]{"cmd.exe", "/c", command});
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }
}
