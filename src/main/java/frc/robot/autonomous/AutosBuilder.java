package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.swerve.Swerve;
import frc.utils.auto.PathPlannerAutoWrapper;

import java.util.List;
import java.util.function.Supplier;

public class AutosBuilder {

	public static List<Supplier<PathPlannerAutoWrapper>> getAllTestAutos() {
		return List.of(
			() -> new PathPlannerAutoWrapper("Rotate"),
			() -> new PathPlannerAutoWrapper("Rotate 2m"),
			() -> new PathPlannerAutoWrapper("Straight 2m")
		);
	}

	public static PathPlannerAutoWrapper createDefaultAuto(Swerve swerve) {
		return new PathPlannerAutoWrapper(
			swerve.getCommandsBuilder()
				.drive(() -> AutonomousConstants.DEFAULT_AUTO_DRIVE_POWER, () -> 0, () -> 0)
				.withTimeout(AutonomousConstants.DEFAULT_AUTO_DRIVE_TIME_SECONDS)
				.andThen(swerve.getCommandsBuilder().resetTargetSpeeds()),
			Pose2d.kZero,
			"Default Auto"
		);
	}

}
