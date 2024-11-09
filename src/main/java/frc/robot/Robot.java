// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.poseestimator.GBPoseEstimator;
import frc.robot.poseestimator.OdometryValues;
import frc.robot.structures.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.factories.SimulationSwerveGenerator;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.gyro.SimulationGyroConstants;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.modules.SimulationModuleGenerator;
import frc.robot.subsystems.swerve.factories.swerveconstants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.swervestatehelpers.SwerveStateHelper;
import frc.robot.vision.MultiVisionSources;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.VisionFilterer;
import frc.robot.vision.VisionFiltererConfig;
import frc.robot.vision.sources.simulationsource.SimulatedSource;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Supplier;

import frc.robot.bindings.KeyboardBindings;
import frc.robot.bindings.JoysticksBindings;
import frc.utils.controllers.keyboard.KeyboardController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	private final SwerveDriveSimulation swerveDriveSimulation;

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final Swerve swerve;
	private final GBPoseEstimator poseEstimator;
	private final Superstructure superStructure;

	public Robot() {
		GyroSimulation gyroSimulation = null;
		if (ROBOT_TYPE.isSimulation()) {
			gyroSimulation = SimulationGyroConstants.generateGyroSimulation();
			Supplier<SwerveModuleSimulation> simulationModule = SimulationModuleGenerator.generate();
			this.swerveDriveSimulation = SimulationSwerveGenerator
				.generate(simulationModule, gyroSimulation, new Pose2d(2, 5, new Rotation2d()));
			SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
		} else {
			swerveDriveSimulation = null;
		}
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(SwerveType.SWERVE),
			ModulesFactory.create(SwerveType.SWERVE, swerveDriveSimulation),
			GyroFactory.create(SwerveType.SWERVE, gyroSimulation)
		);

		this.poseEstimator = new GBPoseEstimator(
			"poseEstimator/",
			new VisionFilterer(
				new VisionFiltererConfig("visionFiltere/", VisionConstants.DEFAULT_VISION_FILTERS_TOLERANCES),
				new MultiVisionSources(
					new SimulatedSource(
						"limelight-front",
						() -> swerveDriveSimulation.getSimulatedDriveTrainPose(),
						VisionConstants.LIMELIGHT_3_SIMULATED_SOURCE_CONFIGURATION
					)
				)
			),
			new OdometryValues(swerve.getConstants().kinematics(), swerve.getModules().getWheelsPositions(0), swerve.getAbsoluteHeading()),
			new double[] {.02, 0.02, 0.03}
		);

		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
		swerve.setStateHelper(new SwerveStateHelper(() -> Optional.of(poseEstimator.getEstimatedPose()), Optional::empty, swerve));

		this.superStructure = new Superstructure(swerve, poseEstimator);

		buildPathPlannerForAuto();
		configureBindings();
	}

	private void buildPathPlannerForAuto() {
		// Register commands...
		swerve.configPathPlanner(poseEstimator::getEstimatedPose, poseEstimator::resetPose);
	}

	private void configureBindings() {
		JoysticksBindings.configureBindings(this);
		if (KeyboardController.ENABLE_KEYBOARD) {
			KeyboardBindings.configureBindings(this);
		}
	}


	public Command getAutonomousCommand() {
		return new InstantCommand();
	}

	public Superstructure getSuperStructure() {
		return superStructure;
	}

	public Swerve getSwerve() {
		return swerve;
	}

	public GBPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

	public void updateSimulationRobot() {
		if (swerveDriveSimulation != null) {
			Logger.recordOutput("FieldSimulation/RobotPosition", swerveDriveSimulation.getSimulatedDriveTrainPose());
		}
	}

}
