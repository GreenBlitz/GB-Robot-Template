// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.RobotManager;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.RobotCommander;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.factory.ArmFactory;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.factory.ElevatorFactory;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.factory.EndEffectorFactory;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.brakestate.BrakeStateManager;
import frc.utils.battery.BatteryUtil;

import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final WPILibPoseEstimatorWrapper poseEstimator;

	private final Swerve swerve;
	private final Elevator elevator;
	private final Arm arm;
	private final EndEffector endEffector;

	private final SimulationManager simulationManager;
	private final RobotCommander robotCommander;

	public Robot() {
		BatteryUtil.scheduleLimiter();

		IGyro gyro = GyroFactory.createGyro(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve");
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			ModulesFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			gyro,
			GyroFactory.createSignals(gyro)
		);

		this.poseEstimator = new WPILibPoseEstimatorWrapper(
			WPILibPoseEstimatorConstants.WPILIB_POSEESTIMATOR_LOGPATH,
			swerve.getKinematics(),
			swerve.getModules().getWheelPositions(0),
			swerve.getGyroAbsoluteYaw()
		);

		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
		swerve.getStateHandler().setRobotPoseSupplier(poseEstimator::getEstimatedPose);

		this.elevator = ElevatorFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Elevator");
		BrakeStateManager.add(() -> elevator.setBrake(true), () -> elevator.setBrake(false));

		this.arm = ArmFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Arm");
		BrakeStateManager.add(() -> arm.setBrake(true), () -> arm.setBrake(false));

		this.endEffector = EndEffectorFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/EndEffector");

		this.simulationManager = new SimulationManager("SimulationManager", this);
		this.robotCommander = new RobotCommander("StateMachine/RobotCommander", this);

		swerve.getStateHandler().setCoralStationSlotSupplier(() -> Optional.of(ScoringHelpers.targetcoralStationSlot));

		configPathPlanner();
	}

	public void configPathPlanner() {
		swerve.configPathPlanner(
			poseEstimator::getEstimatedPose,
			poseEstimator::resetPose,
			PathPlannerUtil.getGuiRobotConfig()
				.orElse(new RobotConfig(70, 0.0001, new ModuleConfig(0.048, 5.24, 0.96, DCMotor.getFalcon500Foc(1), 60, 1), 0.577))
		);
	}

	public void periodic() {
		swerve.update();
		poseEstimator.updateOdometry(swerve.getAllOdometryObservations());
		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		simulationManager.logPoses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public Command getAutonomousCommand() {
		return new InstantCommand();
	}

	public WPILibPoseEstimatorWrapper getPoseEstimator() {
		return poseEstimator;
	}

	public Swerve getSwerve() {
		return swerve;
	}

	public Elevator getElevator() {
		return elevator;
	}

	public Arm getArm() {
		return arm;
	}

	public EndEffector getEndEffector() {
		return endEffector;
	}

	public RobotCommander getRobotCommander() {
		return robotCommander;
	}

}
