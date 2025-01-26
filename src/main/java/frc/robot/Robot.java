// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.RobotManager;
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.hardware.interfaces.IAccelerometer;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.interfaces.IVibrationGyro;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.poseestimation.PoseEstimator;
import frc.robot.structures.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.gyro.PigeonFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.utils.auto.PathPlannerUtils;
import frc.utils.battery.BatteryUtils;
import frc.utils.linearfilters.PeriodicNDimlLinearFilter;
import org.ejml.simple.SimpleMatrix;
import org.littletonrobotics.junction.Logger;

import java.util.List;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final Swerve swerve;
	private final PoseEstimator poseEstimator;
	private final Superstructure superStructure;
	private final IAccelerometer accelerometer;
	private final IVibrationGyro vibrationGyro;
	private final PeriodicNDimlLinearFilter<N3> accelerationFilter;

	public Robot() {
		BatteryUtils.scheduleLimiter();

		IGyro gyro = PigeonFactory.createGyro(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve");
		this.accelerometer = PigeonFactory.createAccelerometer(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Accelerometer");
		this.accelerationFilter = new PeriodicNDimlLinearFilter<>(
			() -> List
				.of(
					new Vector<>(
						new SimpleMatrix(
							new double[][] {
								{accelerometer.getAccelerationX(), accelerometer.getAccelerationY(), accelerometer.getAccelerationZ()}}
						)
					)
				),
			List.of(LinearFilter.movingAverage(10), LinearFilter.movingAverage(10), LinearFilter.movingAverage(10)),
			"3DimAcceleration",
			N3.instance
		);
		this.vibrationGyro = PigeonFactory.createVibrationGyro(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/VibrationGyro");
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			ModulesFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			gyro,
			PigeonFactory.createSignals(gyro)
		);

		this.poseEstimator = new PoseEstimator(swerve::setHeading, swerve.getKinematics());

		swerve.setHeadingSupplier(() -> poseEstimator.getCurrentPose().getRotation());
		swerve.getStateHandler().setRobotPoseSupplier(poseEstimator::getCurrentPose);

		this.superStructure = new Superstructure(swerve, poseEstimator);

		buildPathPlannerForAuto();
	}


	private void buildPathPlannerForAuto() {
		// Register commands...
		swerve.configPathPlanner(
			poseEstimator::getCurrentPose,
			poseEstimator::resetPose,
			PathPlannerUtils.getGuiRobotConfig().orElse(AutonomousConstants.SYNCOPA_ROBOT_CONFIG)
		);
	}


	public void periodic() {
		BatteryUtils.logStatus();
		BusChain.logChainsStatuses();
		superStructure.periodic();
		accelerometer.logAcceleration();
		vibrationGyro.logAngularVelocities();
		Logger.recordOutput("denoisedAcceleration", accelerationFilter.getAsColumnVector());
		CommandScheduler.getInstance().run(); // Should be last
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

}
