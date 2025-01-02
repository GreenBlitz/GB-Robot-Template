package frc.utils.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import java.util.function.Consumer;

public class GBAuto extends PathPlannerAuto {

	GBAuto() {
		super(Commands.none());
	}

	public GBAuto(String autoName) {
		super(autoName);
	}

	public GBAuto(Command autoCommand, Pose2d startingPose, String autoName, boolean isFullyCreated) {
		super(autoCommand, startingPose);
		if (!isFullyCreated) {
			autoName += " (partial)";
		}
		setName(autoName);
	}

	public GBAuto withResetPose(Consumer<Pose2d> resetPose) {
		return new GBAuto(
			new InstantCommand(() -> resetPose.accept(this.getStartingPose())).andThen(this),
			this.getStartingPose(),
			this.getName(),
			true
		);
	}

}
