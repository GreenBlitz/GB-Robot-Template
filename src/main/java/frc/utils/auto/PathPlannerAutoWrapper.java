package frc.utils.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
