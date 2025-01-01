package frc.constants;

import java.nio.file.Path;

public class DirectoryPaths {

	public static final Path REPOSITORY_PATH = Path.of("").toAbsolutePath();
	public static final Path PYTHON_DIRECTORY_PATH = REPOSITORY_PATH.resolve("src/main/python");
	public static final Path JAVA_DIRECTORY_PATH = REPOSITORY_PATH.resolve("src/main/java");
	public static final Path COMPUTER_LOG_FILES_DIRECTORY_PATH = REPOSITORY_PATH.resolve("computerlogfiles");
	public static final Path APRIL_TAG_FIELD_CONFIG_FILE_PATH = REPOSITORY_PATH.resolve("april_tag_field_config.json");

}
