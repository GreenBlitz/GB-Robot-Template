package frc.utils.auto;

import com.pathplanner.lib.path.PathPlannerPath;

import java.util.Optional;

public enum AutoPath {

	AUTO_LINE_2_TO_I("AL2-I"),
	AUTO_LINE_4_TO_H("AL4-H"),
	AUTO_LINE_6_TO_F("AL6-F"),
	I_TO_UPPER_CORAL_STATION("I-US"),
	F_TO_LOWER_CORAL_STATION("F-LS"),
	LOWER_CORAL_STATION_TO_C("LS-C"),
	UPPER_CORAL_STATION_TO_L("US-L");

	private final String pathName;

	AutoPath(String pathName) {
		this.pathName = pathName;
	}

	public String getPathName() {
		return pathName;
	}

	public Optional<PathPlannerPath> getPathOptional() {
		return PathPlannerUtils.getPathFromFile(pathName);
	}

}
