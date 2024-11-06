// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.poseestimation.PoseEstimator;
import frc.robot.poseestimation.PoseEstimatorConstants;
import frc.robot.structures.Superstructure;
import frc.robot.subsystems.intake.MapleIntake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.factories.SimulationSwerveGenerator;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.gyro.SimulationGyroConstants;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.modules.SimulationModuleGenerator;
import frc.robot.subsystems.swerve.factories.swerveconstants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.swervestatehelpers.SwerveStateHelper;
import org.ironmaple.simulation.IntakeSimulation;
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

	private final SwerveDriveSimulation swerveDriveSimulation;

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final Swerve swerve;
	private final MapleIntake mapleIntake;
	private final PoseEstimator poseEstimator;
	private final Superstructure superStructure;

	public Robot() {
		GyroSimulation gyroSimulation = null;
		if (ROBOT_TYPE.isSimulation()) {
			gyroSimulation = SimulationGyroConstants.generateGyroSimulation();
			Supplier<SwerveModuleSimulation> simulationModule = SimulationModuleGenerator.generate();
			this.swerveDriveSimulation = SimulationSwerveGenerator
				.generate(simulationModule, gyroSimulation, PoseEstimatorConstants.DEFAULT_POSE);
			SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
		} else {
			swerveDriveSimulation = null;
		}
		IntakeSimulation intakeSimulation = new IntakeSimulation(
			"Note", // the intake grabs game pieces of this type
			swerveDriveSimulation, // specify the drivetrain to which the intake is attached to
			0.6, // the width of the intake
			IntakeSimulation.IntakeSide.BACK, // the intake is attached the back of the drivetrain
			1 // the intake can only hold 1 game piece at a time
		);
		intakeSimulation.register();
		mapleIntake = new MapleIntake("Subsystems/Intake", intakeSimulation);

		this.swerve = new Swerve(
			SwerveConstantsFactory.create(SwerveType.SWERVE),
			ModulesFactory.create(SwerveType.SWERVE, swerveDriveSimulation),
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

	public void updateSimulationRobot() {
		if (swerveDriveSimulation != null) {
			Logger.recordOutput("FieldSimulation/RobotPosition", swerveDriveSimulation.getSimulatedDriveTrainPose());
		}
		mapleIntake.visualizeNoteInIntake(
			swerveDriveSimulation == null ? poseEstimator.getCurrentPose() : swerveDriveSimulation.getSimulatedDriveTrainPose()
		);
	}

	public Command intake() {
		return new SequentialCommandGroup(
			new RunCommand(() -> mapleIntake.setRunning(true), mapleIntake).until(mapleIntake::isNoteInsideIntake),
			new RunCommand(() -> mapleIntake.setRunning(false), mapleIntake)
		);
	}

}
