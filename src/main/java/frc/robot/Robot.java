// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.hardware.gyro.maple.MapleGyro;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.poseestimation.PoseEstimator;
import frc.robot.poseestimation.PoseEstimatorConstants;
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
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Supplier;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	private static final boolean IS_MAPLE = true;


	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final Swerve swerve;
	private final SwerveDriveSimulation swerveDriveSimulation;
	private final PoseEstimator poseEstimator;
	private final Superstructure superStructure;

	public Robot() {
		IGyro gyro = GyroFactory.createGyro(SwerveType.SWERVE, IS_MAPLE);

		if (ROBOT_TYPE.isSimulation() && IS_MAPLE) { //todo: move into swerve
			GyroSimulation gyroSimulation = ((MapleGyro) gyro).getGyroSimulation();
			Supplier<SwerveModuleSimulation> simulationModule = SimulationModuleGenerator.generate();
			swerveDriveSimulation = SimulationSwerveGenerator.generate(simulationModule, gyroSimulation, PoseEstimatorConstants.DEFAULT_POSE);
			SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
		} else {
			swerveDriveSimulation = null;
		}

		this.swerve = new Swerve(
			SwerveConstantsFactory.create(SwerveType.SWERVE),
<<<<<<< HEAD
			ModulesFactory.create(SwerveType.SWERVE, swerveDriveSimulation),
=======
			ModulesFactory.create(SwerveType.SWERVE),
>>>>>> core-swerve
			gyro,
			GyroFactory.createSignals(SwerveType.SWERVE, gyro)
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

	public void logSimulationRobot() {
		if (swerveDriveSimulation != null) {
			Logger.recordOutput("FieldSimulation/RobotPosition", swerveDriveSimulation.getSimulatedDriveTrainPose());
		}
	}

}
