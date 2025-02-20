package frc.utils.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import frc.constants.field.enums.Branch;

import java.util.Optional;

public enum AutoPath {

	;

	private final Pair<String, Pose2d> startingPoint;
	private final Pair<String, Pose2d> endPoint;
	private final Optional<Branch> targetBranch;

	AutoPath(Pair<String, Pose2d> startingPoint, Pair<String, Pose2d> endPoint, Optional<Branch> targetBranch) {
		this.startingPoint = startingPoint;
		this.endPoint = endPoint;
		this.targetBranch = targetBranch;
	}

	public Pair<String, Pose2d> getStartingPoint() {
		return startingPoint;
	}

	public Pair<String, Pose2d> getEndPoint() {
		return endPoint;
	}

	public Optional<Branch> getTargetBranch() {
		return targetBranch;
	}

	public String getPathName() {
		return startingPoint.getFirst() + "-" + endPoint.getFirst();
	}

	public Optional<PathPlannerPath> getPath() {
		return PathPlannerUtil.getPathFromFile(getPathName());
	}

}
