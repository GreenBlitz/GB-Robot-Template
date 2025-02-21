package frc.utils.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.constants.field.Field;
import frc.utils.math.AngleTransform;

import java.util.function.Consumer;

public class PathPlannerAutoWrapper extends PathPlannerAuto {

	private final boolean isFullyCreated;

	public PathPlannerAutoWrapper() {
		super(Commands.none());
		this.isFullyCreated = true;
	}

	public PathPlannerAutoWrapper(String autoName) {
		super(autoName);
		this.isFullyCreated = true;
	}

	public PathPlannerAutoWrapper(Command autoCommand, Pose2d startingPose, String autoName, boolean isFullyCreated) {
		super(autoCommand, startingPose);
		this.isFullyCreated = isFullyCreated;

		if (!this.isFullyCreated) {
			autoName += " (partial)";
		}
		setName(autoName);
	}

	public PathPlannerAutoWrapper withResetPose(Consumer<Pose2d> resetPose) {
		return new PathPlannerAutoWrapper(
			this.beforeStarting(() -> resetPose.accept(Field.getAllianceRelative(this.getStartingPose(), true, true, AngleTransform.INVERT))),
			this.getStartingPose(),
			this.getName(),
			true
		);
	}

	public boolean isFullyCreated() {
		return isFullyCreated;
	}

	public static PathPlannerAutoWrapper chainAutos(PathPlannerAutoWrapper... autos) {
		return new PathPlannerAutoWrapper(Commands.none().andThen(autos), autos[0].getStartingPose(), chainAutoNames(autos), true);
	}

	private static String chainAutoNames(PathPlannerAutoWrapper... autos) {
		StringBuilder autoName = new StringBuilder();
		for (PathPlannerAutoWrapper auto : autos) {
			autoName.append(auto.getName()).append("-");
		}
		return autoName.deleteCharAt(autoName.length() - 1).toString();
	}

}
