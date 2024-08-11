package frc.utils;

import frc.robot.constants.DirectoryPaths;

import java.nio.file.Path;
import java.util.*;

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
			CMD_COMPUTER_LOG_FILE.write("Trying To Run: " + executedCommand + " At Time: " + Calendar.getInstance().getTime());
			ProcessBuilder builder = new ProcessBuilder(executedCommand);
			builder.redirectErrorStream(true);
			Process process = builder.start();
			new Thread(() -> readOutput(executedCommand.toString(), process)).start();
		} catch (Exception exception) {
			CMD_COMPUTER_LOG_FILE.write("Got Exception: " + exception);
			CMD_COMPUTER_LOG_FILE.open();
		}
	}

	private static void readOutput(String executedCommandName, Process process) {
		StringBuilder cmdOutput = new StringBuilder();
		Scanner scanner = new Scanner(process.getInputStream(), "UTF-8");
		while (scanner.hasNextLine()) {
			cmdOutput.append(scanner.nextLine());
		}
		scanner.close();
		CMD_COMPUTER_LOG_FILE.write("Got cmd Output From " + executedCommandName + ":");
		CMD_COMPUTER_LOG_FILE.write(cmdOutput.toString());
		CMD_COMPUTER_LOG_FILE.write("Successfully Finished Running Command At Time: " + Calendar.getInstance().getTime() + "\n");
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

	public static void runPythonClass(Path pythonPath) {
		runPythonClass(pythonPath, "");
	}

	/**
	 * @param pythonPath The path from the java package to the class. example: "directory/example_class".
	 * @param arguments  The arguments given to the python file.
	 */
	public static void runPythonClass(Path pythonPath, String... arguments) {
		String command = "py " + pythonPath + ".py " + getSeparatedArguments(arguments);
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
