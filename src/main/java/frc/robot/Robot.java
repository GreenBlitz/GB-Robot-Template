// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.factories.imu.IMUFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;
import frc.robot.hardware.interfaces.IIMU;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType(false);

	private Swerve swerve;
	private IPoseEstimator poseEstimator;
	private final TalonFXSysid talonFXSysid;

	public Robot() {
		BatteryUtil.scheduleLimiter();

//		IIMU imu = IMUFactory.createIMU(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve");
//		this.swerve = new Swerve(
//			SwerveConstantsFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
//			ModulesFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
//			imu,
//			IMUFactory.createSignals(imu)
//		);

//		this.poseEstimator = new WPILibPoseEstimatorWrapper(
//			WPILibPoseEstimatorConstants.WPILIB_POSEESTIMATOR_LOGPATH,
//			swerve.getKinematics(),
//			swerve.getModules().getWheelPositions(0),
//			swerve.getGyroAbsoluteYaw().getValue(),
//			swerve.getGyroAbsoluteYaw().getTimestamp()
//		);

		this.talonFXSysid = new TalonFXSysid(new Phoenix6DeviceID(10, BusChain.ROBORIO), 1, 7);

//		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
	}

	public void periodic() {
		BusChain.refreshAll();

//		swerve.update();
//		poseEstimator.updateOdometry(swerve.getAllOdometryData());
//		poseEstimator.log();

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public IPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

	public Swerve getSwerve() {
		return swerve;
	}

	public PathPlannerAutoWrapper getAutonomousCommand() {
		return new PathPlannerAutoWrapper();
	}

	public TalonFXSysid getTalonFXSysid() {
		return talonFXSysid;
	}

}
