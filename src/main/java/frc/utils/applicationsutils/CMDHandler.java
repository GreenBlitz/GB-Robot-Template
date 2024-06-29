package frc.utils.applicationsutils;

import java.io.IOException;
import java.nio.file.Path;

public class CMDHandler {

    public static final Path REPOSITORY_PATH = Path.of("").toAbsolutePath();
    public static final Path PATH_TO_PYTHON_DIRECTORY = REPOSITORY_PATH.resolve("src/main/python");
    public static final Path PATH_TO_JAVA_DIRECTORY = REPOSITORY_PATH.resolve("src/main/java");

    private static final String APPLICATION = "cmd.exe";
    private static final String CMD_DIRECTORY = "/c";
    private static final String ERROR_MESSAGE = "Unable to execute: ";


    public static void runCMDCommand(Path directory, String command) {
        runCMDCommand("cd " + directory + "&&" + command);
    }

    public static void runCMDCommand(String command) {
        Runtime runtime = Runtime.getRuntime();
        try {
            runtime.exec(new String[]{APPLICATION, CMD_DIRECTORY, command});
        }
        catch (IOException exception) {
            System.out.println(ERROR_MESSAGE + command);// can't be logged because on computer side
        }
    }


    public static void runJavaClass(Path javaPath) {
        runJavaClass(javaPath, "");
    }

    /**
     * @param javaPath The path from the java package to the class. example: "directory/to/my/Example".
     * @param arguments The arguments given to the java file, each argument separated by a space.
     */
    public static void runJavaClass(Path javaPath, String arguments) {
        Path className = javaPath.getName(javaPath.getNameCount() - 1);
        Path packageName = javaPath.getParent();
        runCMDCommand(PATH_TO_JAVA_DIRECTORY.resolve(packageName), "java " + className + ".java " + arguments);
    }


    public static void runJavaClass(Class classToRun) {
        runJavaClass(classToRun, "");
    }

    /**
     * @param classToRun The class to run. example: Example.class .
     * @param arguments The arguments given to the java file, each argument separated by a space.
     */
    public static void runJavaClass(Class classToRun, String arguments) {
        String className = classToRun.getName();
        className = className.replace('.', '/');
        Path pathOfClass = Path.of(className);
        runJavaClass(pathOfClass, arguments);
    }


    public static void runPythonClass(Path pythonPath) {
        runPythonClass(pythonPath, "");
    }

    /**
     * @param pythonPath The path from the java package to the class. example: "directory/example_class".
     * @param arguments The arguments given to the python file, each argument separated by a space.
     */
    public static void runPythonClass(Path pythonPath, String arguments) {
        runCMDCommand(PATH_TO_PYTHON_DIRECTORY, "py " + pythonPath + ".py " + arguments);
    }

}
