// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.JoysticksBindings;
import frc.RobotManager;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.structures.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.swerveconstants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.swervestatehelpers.SwerveStateHelper;
import frc.robot.vision.multivisionsources.MultiAprilTagVisionSources;
import frc.utils.auto.PathPlannerUtils;
import frc.utils.battery.BatteryUtils;

import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final Swerve swerve;
	private final IPoseEstimator poseEstimator;
	private final MultiAprilTagVisionSources aprilTagVisionSources;
	private final Superstructure superStructure;

	public Robot() {
		BatteryUtils.scheduleLimiter();

		IGyro gyro = GyroFactory.createGyro(SwerveType.SWERVE);
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(SwerveType.SWERVE),
			ModulesFactory.create(SwerveType.SWERVE),
			gyro,
			GyroFactory.createSignals(SwerveType.SWERVE, gyro)
		);

		this.poseEstimator = new WPILibPoseEstimator(
			WPILibPoseEstimatorConstants.WPILIB_POSEESTIMATOR_LOGPATH,
			swerve.getKinematics(),
			swerve.getAllOdometryObservations()[0].wheelPositions()
		);

		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
		swerve.setStateHelper(new SwerveStateHelper(() -> Optional.of(poseEstimator.getEstimatedPose()), Optional::empty, swerve));

		this.aprilTagVisionSources = new MultiAprilTagVisionSources(
			"",
			swerve::getGyroAbsoluteYaw,
			() -> poseEstimator.getEstimatedPose().getRotation()
		);

		this.superStructure = new Superstructure(swerve, poseEstimator);

		buildPathPlannerForAuto();
	}


	private void buildPathPlannerForAuto() {
		// Register commands...
		swerve.configPathPlanner(poseEstimator::getEstimatedPose, poseEstimator::resetPose, PathPlannerUtils.SYNCOPA_ROBOT_CONFIG);
	}


	public void periodic() {
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		superStructure.periodic();
		CommandScheduler.getInstance().run();
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

	public IPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

	public MultiAprilTagVisionSources getAprilTagVisionSources() {
		return aprilTagVisionSources;
	}

}
