package frc.utils.applicationsutils;

import frc.robot.constants.DirectoryPathsConstants;

import java.io.File;
import java.nio.file.Path;

public class CMDHandler {

    private static final File OUTPUT_FILE = DirectoryPathsConstants.RUNNING_FILES_DIRECTORY_PATH.resolve("CMDHandler.txt").toFile();
    private static final String WINDOWS_SHELL = "cmd.exe /c ";
    private static final String NON_WINDOWS_SHELL = "bash -c ";


    public static boolean isWindows() {
        return System.getProperty("os.name").toLowerCase().contains("win");
    }

    public static void runCMDCommand(Path directory, String command) {
        runCMDCommand("cd \"" + directory.toString() + "\" && " + command);
    }

    public static void runCMDCommand(String command) {
        String operatingSystemShell = isWindows() ? WINDOWS_SHELL : NON_WINDOWS_SHELL;
        command = operatingSystemShell + command;

        FileCreator.writeToOutputFile(OUTPUT_FILE, "\nRunning: " + command, true);

        Runtime runtime = Runtime.getRuntime();
        try {
            runtime.exec(command);
        }
        catch (Exception exception) {
            FileCreator.writeToOutputFile(OUTPUT_FILE, "\n\nGot Exception: \n" + exception, true);
            FileCreator.openFile(OUTPUT_FILE);
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
        runCMDCommand(DirectoryPathsConstants.JAVA_DIRECTORY_PATH.resolve(packageName), "java " + className + ".java " + arguments);
    }


    public static void runJavaClass(Class<?> classToRun) {
        runJavaClass(classToRun, "");
    }

    /**
     * @param classToRun The class to run. example: Example.class .
     * @param arguments The arguments given to the java file, each argument separated by a space.
     */
    public static void runJavaClass(Class<?> classToRun, String arguments) {
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
        runCMDCommand(DirectoryPathsConstants.PYTHON_DIRECTORY_PATH, "py " + pythonPath + ".py " + arguments);
    }

}
