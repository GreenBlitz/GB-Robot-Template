package frc.robot.subsystems.swerve.factories;

import edu.wpi.first.math.geometry.Pose2d;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

import java.util.function.Supplier;

public class SimulationSwerveGenerator {

	private static final double ROBOT_MASS_WIDTH_BUMPERS_KG = 45;
	private static final double TRACK_WIDTH_METERS = 0.65;
	private static final double TRACK_LENGTH_METERS = 0.65;
	private static final double BUMPER_WIDTH_METERS = 0.74;
	private static final double BUMPER_LENGTH_METERS = 0.74;

	public static SwerveDriveSimulation generate(
		Supplier<SwerveModuleSimulation> moduleSimulationSupplier,
		GyroSimulation gyroSimulation,
		Pose2d startingPose
	) {
		return new SwerveDriveSimulation(
			ROBOT_MASS_WIDTH_BUMPERS_KG,
			TRACK_WIDTH_METERS,
			TRACK_LENGTH_METERS,
			BUMPER_WIDTH_METERS,
			BUMPER_LENGTH_METERS,
			moduleSimulationSupplier,
			gyroSimulation,
			startingPose
		);
	}

}
