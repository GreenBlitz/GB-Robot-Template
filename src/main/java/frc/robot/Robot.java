// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.poseestimation.PoseEstimator;
import frc.robot.structures.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.gyro.SimulationGyroConstants;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.swerveconstants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.module.MapleModule;
import frc.robot.subsystems.swerve.module.Modules;
import frc.robot.subsystems.swerve.swervestatehelpers.SwerveStateHelper;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Optional;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	private final SwerveDriveSimulation swerveDriveSimulation;


	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final Swerve swerve;
	private final PoseEstimator poseEstimator;
	private final Superstructure superStructure;

	public Robot() {
		GyroSimulation gyroSimulation = SimulationGyroConstants.generateGyroSimulation();

		this.swerveDriveSimulation = new SwerveDriveSimulation(
			45,
			0.65,
			0.65,
			0.74,
			0.74,
			SwerveModuleSimulation.getMark4(
				// creates a mark4 module
				DCMotor.getKrakenX60(1), // drive motor is a Kraken x60
				DCMotor.getFalcon500(1), // steer motor is a Falcon 500
				80, // current limit: 80 Amps
				SwerveModuleSimulation.DRIVE_WHEEL_TYPE.RUBBER, // rubber wheels
				3 // l3 gear ratio
			),
			gyroSimulation,
			new Pose2d(
				// initial starting pose on field, set it to where-ever you want
				3,
				3,
				new Rotation2d()
			)
		);
		SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation); // register the drive train simulation

		// reset the field for auto (placing game-pieces in positions)
		SimulatedArena.getInstance().resetFieldForAuto();

		Modules modules = ROBOT_TYPE.isReal()
			? ModulesFactory.create(SwerveType.SWERVE)
			: new Modules(
				SwerveType.SWERVE.getLogPath(),
				new MapleModule(SwerveType.SWERVE.getLogPath() + "modules/FrontLeft", swerveDriveSimulation.getModules()[0], 0.048359 * 2),
				new MapleModule(SwerveType.SWERVE.getLogPath() + "modules/FrontRight", swerveDriveSimulation.getModules()[1], 0.048359 * 2),
				new MapleModule(SwerveType.SWERVE.getLogPath() + "modules/BackLeft", swerveDriveSimulation.getModules()[2], 0.048359 * 2),
				new MapleModule(SwerveType.SWERVE.getLogPath() + "modules/BackRight", swerveDriveSimulation.getModules()[3], 0.048359 * 2)
			);
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(SwerveType.SWERVE),
			modules,
			GyroFactory.create(SwerveType.SWERVE, gyroSimulation)
		);
		this.poseEstimator = new PoseEstimator(swerve::setHeading, swerve.getConstants().kinematics());

		swerve.setHeadingSupplier(() -> poseEstimator.getCurrentPose().getRotation());
		swerve.setStateHelper(new SwerveStateHelper(() -> Optional.of(poseEstimator.getCurrentPose()), Optional::empty, swerve));

		this.superStructure = new Superstructure(swerve, poseEstimator);

		buildPathPlannerForAuto();
		configureBindings();
	}

	private void buildPathPlannerForAuto() {
		// Register commands...
		swerve.configPathPlanner(poseEstimator::getCurrentPose, poseEstimator::resetPose);
	}

	private void configureBindings() {
		JoysticksBindings.configureBindings(this);
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

	public PoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

	public void updateSimulationField() {
		if (swerveDriveSimulation != null) {
			SimulatedArena.getInstance().simulationPeriodic();

			Logger.recordOutput("FieldSimulation/RobotPosition", swerveDriveSimulation.getSimulatedDriveTrainPose());

			final List<Pose3d> notes = SimulatedArena.getInstance().getGamePiecesByType("Note");
			if (notes != null)
				Logger.recordOutput("FieldSimulation/Notes", notes.toArray(Pose3d[]::new));
		}

//		intake.visualizeNoteInIntake(
//			swerveDriveSimulation == null
//				? drive.getPose()
//				: swerveDriveSimulation.getSimulatedDriveTrainPose());
	}

}
