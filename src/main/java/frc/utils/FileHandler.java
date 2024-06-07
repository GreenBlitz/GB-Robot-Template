package frc.utils;

import java.io.IOException;
import java.nio.file.Path;

    public class FileHandler {

    public static String getPathToPythonDirectory() {
        String repoPath = Path.of("").toAbsolutePath().toString();
        String pythonPath = repoPath + "/src/main/python/";

        return pythonPath;
    }

    public static void runCmd(String command) {
        Runtime rt = Runtime.getRuntime();
        try {
            System.out.println("py " + getPathToPythonDirectory() + "keyboard_to_nt.py");
            rt.exec(new String[]{"cmd.exe", "/c", command});
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }
}
