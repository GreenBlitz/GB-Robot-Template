package frc.utils;

import frc.robot.constants.DirectoryPaths;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

public class CMDHandler {

	private static final ComputerLogFile CMD_COMPUTER_LOG_FILE = new ComputerLogFile("CMDHandler");
	private static final List<String> WINDOWS_SHELL = List.of("cmd.exe", "/c");
	private static final List<String> NON_WINDOWS_SHELL = List.of("bash", "-c");


	public static boolean isWindows() {
		return System.getProperty("os.name").toLowerCase().contains("win");
	}

	public static void runCMDCommand(Path directory, String command) {
		runCMDCommand("cd \"" + directory.toString() + "\" && " + command);
	}

	public static void runCMDCommand(String command) {
		ArrayList<String> executedCommand = new ArrayList<>(isWindows() ? WINDOWS_SHELL : NON_WINDOWS_SHELL);
		executedCommand.add(command);

		try {
			CMD_COMPUTER_LOG_FILE.write("Trying To Run: " + executedCommand);
			new ProcessBuilder(executedCommand).start();
			CMD_COMPUTER_LOG_FILE.write("Success!");
		} catch (Exception exception) {
			CMD_COMPUTER_LOG_FILE.write("Got Exception: \n" + exception);
			CMD_COMPUTER_LOG_FILE.open();
		}
	}

	public static void runJavaClass(Path javaPath) {
		runJavaClass(javaPath, "");
	}

	/**
	 * @param javaPath  The path from the java package to the class. example: "directory/to/my/Example".
	 * @param arguments The arguments given to the java file.
	 */
	public static void runJavaClass(Path javaPath, String... arguments) {
		Path className = javaPath.getName(javaPath.getNameCount() - 1);
		Path packageName = javaPath.getParent();
		String command = "java " + className + ".java " + getSeparatedArguments(arguments);
		runCMDCommand(DirectoryPaths.JAVA_DIRECTORY_PATH.resolve(packageName), command);
	}

	public static void runJavaClass(Class<?> classToRun) {
		runJavaClass(classToRun, "");
	}

	/**
	 * @param classToRun The class to run. example: Example.class .
	 * @param arguments  The arguments given to the java file.
	 */
	public static void runJavaClass(Class<?> classToRun, String... arguments) {
		String className = classToRun.getName();
		className = className.replace('.', '/');
		Path pathOfClass = Path.of(className);
		runJavaClass(pathOfClass, getSeparatedArguments(arguments));
	}

	public static void runPythonScript(Path pythonPath) {
		runPythonScript(pythonPath, "");
	}

	/**
	 * @param pythonPath The path from the java package to the class. example: "directory/example_class".
	 * @param arguments  The arguments given to the python file.
	 */
	public static void runPythonScript(Path pythonPath, String... arguments) {
		String pythonName = isWindows() ? "py " : "python ";
		String command = pythonName + pythonPath + ".py " + getSeparatedArguments(arguments);
		runCMDCommand(DirectoryPaths.PYTHON_DIRECTORY_PATH, command);
	}

	private static String getSeparatedArguments(String[] arguments) {
		StringBuilder separatedArguments = new StringBuilder();
		for (String argument : arguments) {
			separatedArguments.append(argument).append(" ");
		}
		return separatedArguments.toString();
	}

}
