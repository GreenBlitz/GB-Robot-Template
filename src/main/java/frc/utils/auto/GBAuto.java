package frc.utils.auto;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

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

	public GBAuto(GBAuto... autos) {
		this(Commands.none().andThen(autos), autos[0].getStartingPose(), chainAutoNames(autos), true);
	}

	public GBAuto withResetPose(Consumer<Pose2d> resetPose) {
		return new GBAuto(this.beforeStarting(() -> resetPose.accept(this.getStartingPose())), this.getStartingPose(), this.getName(), true);
	}

	private static String chainAutoNames(GBAuto... autos) {
		StringBuilder autoName = new StringBuilder();
		for (GBAuto auto : autos) {
			autoName.append(auto.getName()).append("-");
		}
		return autoName.deleteCharAt(autoName.length()-1).toString();
	}

}
