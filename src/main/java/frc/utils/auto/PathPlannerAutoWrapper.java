package frc.utils.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.constants.field.Field;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class PathPlannerAutoWrapper extends PathPlannerAuto {

	private PathPlannerPath[] paths;

	public PathPlannerAutoWrapper() {
		super(Commands.none());
	}

	public PathPlannerAutoWrapper(String autoName) {
		super(autoName);
	}

	public PathPlannerAutoWrapper(
		Command autoCommand,
		Pose2d startingPose,
		String autoName,
		PathPlannerPath path,
		PathPlannerPath... additionalPaths
	) {
		super(autoCommand, startingPose);
		setName(autoName);

		this.paths = new PathPlannerPath[additionalPaths.length + 1];
		paths[0] = path;
		System.arraycopy(additionalPaths, 0, paths, 1, additionalPaths.length);
	}

	private PathPlannerAutoWrapper(Command autoCommand, Pose2d startingPose, String autoName, PathPlannerPath[] paths) {
		super(autoCommand, startingPose);
		setName(autoName);

		this.paths = paths;
	}

	public PathPlannerAutoWrapper withAutoName(String name) {
		this.setName(name);
		return this;
	}

	public PathPlannerAutoWrapper withResetPose(Consumer<Pose2d> resetPose) {
		return new PathPlannerAutoWrapper(
			this.beforeStarting(() -> resetPose.accept(Field.getAllianceRelative(getStartingPose()))),
			this.getStartingPose(),
			this.getName(),
			this.paths
		);
	}

	public PathPlannerAutoWrapper asProxyAuto() {
		return new PathPlannerAutoWrapper(this.asProxy(), this.getStartingPose(), this.getName(), this.paths);
	}

	public List<Pose2d> getPath(boolean flip) {
		List<Pose2d> finalList = new ArrayList<>();
		if (paths != null) {
			for (PathPlannerPath path : paths) {
				if (flip) {
					path = path.flipPath();
				}
				finalList.addAll(path.getPathPoses());
			}
		}
		return finalList;
	}


}
