package frc.utils.auto;

import com.pathplanner.lib.path.PathPlannerPath;

import java.util.Optional;

public enum AutoPath {

	MIDDLE_OF_SUBWOOFER_TO_NOTE_2("M2"),
	NOTE_2_TO_NOTE_3("23"),
	NOTE_3_TO_NOTE_1("31");

	private final String pathName;

	AutoPath(String pathName) {
		this.pathName = pathName;
	}

	public Optional<PathPlannerPath> getPathOptional() {
		return PathPlannerUtils.getPathFromFile(pathName);
	}

}
