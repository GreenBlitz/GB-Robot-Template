// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;
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

	public Command shoot() {
		return new ParallelCommandGroup(
			new InstantCommand(mapleIntake::releaseNote, mapleIntake),
			new InstantCommand(() -> shootNoteWithCurrentRPM(
				swerveDriveSimulation.getSimulatedDriveTrainPose(),
				swerveDriveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
				Rotation2d.fromRotations(60 * 60)
			))
		);
	}

	public void shootNoteWithCurrentRPM(Pose2d robotSimulationWorldPose, ChassisSpeeds chassisSpeedsFieldRelative,
		Rotation2d velocityMinutes) {
		SimulatedArena.getInstance()
					  .addGamePieceProjectile(
						  new NoteOnFly(
							  robotSimulationWorldPose
								  .getTranslation(), // specify the position of the chassis
							  new Translation2d(
								  0.2,
								  0), // the shooter is installed at this position on the robot (in reference
							  // to the robot chassis center)
							  chassisSpeedsFieldRelative, // specify the field-relative speed of the chassis
							  // to add it to the initial velocity of the projectile
							  robotSimulationWorldPose
								  .getRotation(), // the shooter facing is the robot's facing
							  0.45, // initial height of the flying note
							  velocityMinutes.getRotations()
								  / 6000
								  * 20, // we think the launching speed is proportional to the rpm, and is 16
							  // meters/second when the motor rpm is 6000
							  Math.toRadians(55) // the note is launched at fixed angle of 55 degrees.
						  )
							  .asSpeakerShotNote(() -> System.out.println("hit target!!!"))
							  .enableBecomeNoteOnFieldAfterTouchGround()
							  .withProjectileTrajectoryDisplayCallBack(
								  (pose3ds) ->
									  Logger.recordOutput(
										  "Flywheel/NoteProjectileSuccessful", pose3ds.toArray(Pose3d[]::new)),
								  (pose3ds) ->
									  Logger.recordOutput(
										  "Flywheel/NoteProjectileUnsuccessful",
										  pose3ds.toArray(Pose3d[]::new))));
	}

}
