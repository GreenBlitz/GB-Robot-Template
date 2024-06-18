package frc.utils.applicationsutils;

import java.io.IOException;
import java.nio.file.Path;

public class CMDHandler {

    public static final String REPOSITORY_PATH = Path.of("").toAbsolutePath().toString();
    public static final String PATH_TO_PYTHON_DIRECTORY = REPOSITORY_PATH + "/src/main/python/";
    public static final String PATH_TO_JAVA_DIRECTORY = REPOSITORY_PATH + "/src/main/java/";

    private static final String APPLICATION = "cmd.exe";
    private static final String CMD_DEFAULT_DIRECTORY = "/c";
    private static final String ERROR_MESSAGE = "Unable to execute: ";

    public static void runCMDCommand(String command) {
        runCMDCommand(CMD_DEFAULT_DIRECTORY, command);
    }

    public static void runCMDCommand(String directory, String command) {
        Runtime runtime = Runtime.getRuntime();
        try {
            runtime.exec(new String[]{APPLICATION, directory, command});
        } catch (IOException exception) {
            System.out.println(ERROR_MESSAGE + command);// can't be logged because on computer side
        }
    }

    /**
     * @param javaPath The path from the java package to the class.
     *                 example: "frc/utils/applicationsutils/CMDHandler.java"
     */
    public static void runJavaClass(String javaPath) {
        int lastSlash = javaPath.lastIndexOf('/');
        String className = javaPath.substring(lastSlash);
        String packageName = javaPath.substring(0, lastSlash);
        runCMDCommand(packageName ,"java " + className);
    }

    /**
     * @param classToRun The class to run. example: CMDHandler.class
     */
    public static void runJavaClass(Class classToRun) {
        String className = classToRun.getName();
        className = className.replace('.','/');
        className += ".java";
        runJavaClass(className);
    }

    /**
     * @param pythonPath The path from the java package to the class.
     *                   example: "keyboard/keyboard_to_nt.py"
     */
    public static void runPythonClass(String pythonPath) {
        runCMDCommand("py " + PATH_TO_PYTHON_DIRECTORY + pythonPath);
    }
}
