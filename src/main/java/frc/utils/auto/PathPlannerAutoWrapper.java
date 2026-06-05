package frc.utils.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import frc.constants.field.Field;

import java.util.function.Consumer;

public class PathPlannerAutoWrapper extends PathPlannerAuto {

	public PathPlannerAutoWrapper() {
		super(Commands.none());
	}

	public PathPlannerAutoWrapper(String autoName) {
		super(autoName);
	}

	public PathPlannerAutoWrapper(Command autoCommand, Pose2d startingPose, String autoName) {
		super(autoCommand, startingPose);
		setName(autoName);
	}

	public PathPlannerAutoWrapper withAutoName(String name) {
		this.setName(name);
		return this;
	}

	public PathPlannerAutoWrapper withResetPose(Consumer<Pose2d> resetPose) {
		return new PathPlannerAutoWrapper(
			this.beforeStarting(() -> resetPose.accept(Field.getAllianceRelative(getStartingPose()))),
			this.getStartingPose(),
			this.getName()
		);
	}

	public PathPlannerAutoWrapper asProxyAuto() {
		return new PathPlannerAutoWrapper(this.asProxy(), this.getStartingPose(), this.getName());
	}

}
