package frc.utils.applicationsutils;

import frc.robot.constants.DirectoryPathsConstants;

import java.nio.file.Path;

public class CMDHandler {

    private static final OutputFile CMD_OUTPUT_FILE = new OutputFile("CMDHandler.txt");
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
        String executedCommand = operatingSystemShell + command;

        try {
            CMD_OUTPUT_FILE.write("Trying To Run: " + executedCommand);
            Runtime.getRuntime().exec(executedCommand);
        }
        catch (Exception exception) {
            CMD_OUTPUT_FILE.write("\nGot Exception: \n" + exception);
            CMD_OUTPUT_FILE.open();
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
        String command = "java " + className + ".java " + arguments;
        runCMDCommand(DirectoryPathsConstants.JAVA_DIRECTORY_PATH.resolve(packageName), command);
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
        String command = "py " + pythonPath + ".py " + arguments;
        runCMDCommand(DirectoryPathsConstants.PYTHON_DIRECTORY_PATH, command);
    }

}
