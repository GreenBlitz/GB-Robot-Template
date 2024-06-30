package frc.robot.constants;

import java.nio.file.Path;

public class DirectoryPathsConstants {

    public static final Path REPOSITORY_PATH = Path.of("").toAbsolutePath();
    public static final Path PYTHON_DIRECTORY_PATH = REPOSITORY_PATH.resolve("src/main/python");
    public static final Path JAVA_DIRECTORY_PATH = REPOSITORY_PATH.resolve("src/main/java");
    public static final Path OUTPUT_FILES_DIRECTORY_PATH = REPOSITORY_PATH.resolve("outputfiles");

}
