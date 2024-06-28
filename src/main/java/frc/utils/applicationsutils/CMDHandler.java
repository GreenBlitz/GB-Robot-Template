package frc.utils.applicationsutils;

import java.io.IOException;
import java.nio.file.Path;

public class CMDHandler {

    public static final String REPOSITORY_PATH = Path.of("").toAbsolutePath().toString();
    public static final String PATH_TO_PYTHON_DIRECTORY = REPOSITORY_PATH + "/src/main/python";
    public static final String PATH_TO_JAVA_DIRECTORY = REPOSITORY_PATH + "/src/main/java";

    private static final String APPLICATION = "cmd.exe";
    private static final String CMD_DIRECTORY = "/c";
    private static final String ERROR_MESSAGE = "Unable to execute: ";

    public static void runCMDCommand(String command) {
        Runtime runtime = Runtime.getRuntime();
        System.out.println(command);
        try {
            runtime.exec(new String[]{APPLICATION, CMD_DIRECTORY, command});
        } catch (IOException exception) {
            System.out.println(ERROR_MESSAGE + command);// can't be logged because on computer side
        }
    }

    public static void runCMDCommand(String directory, String command) {
        runCMDCommand("cd " + directory + "&&" + command);
    }

    /**
     * @param javaPath The path from the java package to the class.
     *                 example: "frc/utils/applicationsutils/CMDHandler"
     */
    public static void runJavaClass(String javaPath) {
        int lastSlash = javaPath.lastIndexOf('/');
        String className = javaPath.substring(lastSlash + 1);
        String packageName = javaPath.substring(0, lastSlash);
        runCMDCommand(PATH_TO_JAVA_DIRECTORY + "/" + packageName, "java " + className + ".java");
    }

    /**
     * @param classToRun The class to run. example: CMDHandler.class
     */
    public static void runJavaClass(Class classToRun) {
        String className = classToRun.getName();
        className = className.replace('.', '/');
        runJavaClass(className);
    }

    /**
     * @param pythonPath The path from the java package to the class.
     *                   example: "keyboard/keyboard_to_nt_router"
     */
    public static void runPythonClass(String pythonPath) {
        runCMDCommand(PATH_TO_PYTHON_DIRECTORY, "py " + pythonPath + ".py");
    }

    /**
     * @param pythonPath The path from the java package to the class.
     *                   example: "keyboard/keyboard_to_nt_router"
     *
     * @param arguments The arguments given to the python file, each argument separated by a space.
     *                  Note that it is only possible to transfer strings into the python class.
     *                  Also note that when using the arguments in the python class,
     *                  the first one will always be the path of the class.
     */
    public static void runPythonClass(String pythonPath, String arguments) {
        runCMDCommand(PATH_TO_PYTHON_DIRECTORY, "py " + pythonPath + ".py " + arguments);
    }
}
