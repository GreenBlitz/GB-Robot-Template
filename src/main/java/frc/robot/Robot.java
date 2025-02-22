// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.RobotManager;
import frc.robot.poseestimator.helpers.RobotHeadingEstimator.RobotHeadingEstimatorConstants;
import frc.robot.scoringhelpers.ButtonDriverHelper;
import frc.robot.subsystems.climb.lifter.Lifter;
import frc.robot.subsystems.climb.lifter.factory.LifterFactory;
import frc.robot.subsystems.swerve.factories.modules.drive.KrakenX60DriveBuilder;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.robot.vision.VisionConstants;
import frc.robot.hardware.interfaces.IGyro;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.poseestimator.helpers.RobotHeadingEstimator.RobotHeadingEstimator;
import frc.robot.statemachine.RobotCommander;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.factory.ArmFactory;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.factory.ElevatorFactory;
import frc.robot.subsystems.endeffector.EndEffector;
import frc.robot.subsystems.endeffector.factory.EndEffectorFactory;
import frc.robot.subsystems.climb.solenoid.Solenoid;
import frc.robot.subsystems.climb.solenoid.factory.SolenoidFactory;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.vision.VisionFilters;
import frc.robot.vision.data.VisionData;
import frc.robot.vision.multivisionsources.MultiAprilTagVisionSources;
import frc.utils.Filter;
import frc.utils.TimedValue;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.brakestate.BrakeStateManager;
import frc.utils.battery.BatteryUtil;
import frc.utils.time.TimeUtil;

import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private final IPoseEstimator poseEstimator;
	private final RobotHeadingEstimator headingEstimator;
	private final MultiAprilTagVisionSources multiAprilTagVisionSources;

	private final Swerve swerve;
	private final Elevator elevator;
	private final Arm arm;
	private final EndEffector endEffector;
	private final Solenoid solenoid;
	private final Lifter lifter;

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

		this.headingEstimator = new RobotHeadingEstimator(
			RobotHeadingEstimatorConstants.DEFAULT_HEADING_ESTIMATOR_LOGPATH,
			new Rotation2d(),
			new Rotation2d(),
			RobotHeadingEstimatorConstants.DEFAULT_GYRO_STANDARD_DEVIATION
		);

		this.multiAprilTagVisionSources = new MultiAprilTagVisionSources(
			VisionConstants.MULTI_VISION_SOURCES_LOGPATH,
			headingEstimator::getEstimatedHeading,
			true,
			VisionConstants.VISION_SOURCES
		);

		multiAprilTagVisionSources.applyFunctionOnAllFilters(
			filters -> filters.and(
				new Filter<>(
					data -> VisionFilters.isYawAtAngle(headingEstimator::getEstimatedHeading, VisionConstants.YAW_FILTER_TOLERANCE)
						.apply((VisionData) data)
				)
			)
		);

		swerve.setHeadingSupplier(
			ROBOT_TYPE.isSimulation() ? () -> poseEstimator.getEstimatedPose().getRotation() : headingEstimator::getEstimatedHeading
		);

		swerve.getStateHandler().setRobotPoseSupplier(poseEstimator::getEstimatedPose);
		swerve.getStateHandler().setBranchSupplier(() -> Optional.of(ScoringHelpers.getTargetBranch()));
		swerve.getStateHandler().setReefSideSupplier(() -> Optional.of(ScoringHelpers.getTargetReefSide()));
		swerve.getStateHandler().setCoralStationSupplier(() -> Optional.of(ScoringHelpers.getTargetCoralStation(this)));

		this.elevator = ElevatorFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Elevator");
		BrakeStateManager.add(() -> elevator.setBrake(true), () -> elevator.setBrake(false));

		this.arm = ArmFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Arm");
		BrakeStateManager.add(() -> arm.setBrake(true), () -> arm.setBrake(false));

		this.endEffector = EndEffectorFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/EndEffector");

		this.solenoid = SolenoidFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Solenoid");

		this.lifter = LifterFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Lifter");
		BrakeStateManager.add(() -> lifter.setBrake(true), () -> lifter.setBrake(false));

		this.simulationManager = new SimulationManager("SimulationManager", this);
		this.robotCommander = new RobotCommander("StateMachine/RobotCommander", this);

		configPathPlanner();
	}

	public void configPathPlanner() {
		swerve.configPathPlanner(
			poseEstimator::getEstimatedPose,
			poseEstimator::resetPose,
			PathPlannerUtil.getGuiRobotConfig().orElse(getRobotConfig())
		);
	}

	public void periodic() {
		swerve.update();
		arm.setReversedSoftLimit(robotCommander.getSuperstructure().getArmReversedSoftLimitByElevator());

		poseEstimator.updateOdometry(swerve.getAllOdometryData());
		headingEstimator.updateGyroAngle(new TimedValue<>(swerve.getGyroAbsoluteYaw(), TimeUtil.getCurrentTimeSeconds()));
		for (TimedValue<Rotation2d> headingData : multiAprilTagVisionSources.getRawRobotHeadings()) {
			headingEstimator.updateVisionIfGyroOffsetIsNotCalibrated(
				headingData,
				RobotHeadingEstimatorConstants.DEFAULT_VISION_STANDARD_DEVIATION,
				RobotHeadingEstimatorConstants.MAXIMUM_STANDARD_DEVIATION_TOLERANCE
			);
		}
		poseEstimator.updateVision(multiAprilTagVisionSources.getFilteredVisionData());
		multiAprilTagVisionSources.log();
		headingEstimator.log();

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		simulationManager.logPoses();
		ScoringHelpers.log("Scoring");
		ButtonDriverHelper.log("Scoring/ButtonDriverDisplay");

		CommandScheduler.getInstance().run(); // Should be last
	}

	public Command getAutonomousCommand() {
		return new InstantCommand();
	}

	public IPoseEstimator getPoseEstimator() {
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

	public Solenoid getSolenoid() {
		return solenoid;
	}

	public Lifter getLifter() {
		return lifter;
	}

	public RobotCommander getRobotCommander() {
		return robotCommander;
	}

	public RobotConfig getRobotConfig() {
		return new RobotConfig(
			RobotConstants.MASS_KILOGRAM,
			RobotConstants.MOMENT_OF_INERTIA_KILOGRAM_METERS_SQUARED,
			new ModuleConfig(
				swerve.getModules().getModule(ModuleUtil.ModulePosition.FRONT_LEFT).getModuleConstants().wheelDiameterMeters() / 2,
				swerve.getConstants().velocityAt12VoltsMetersPerSecond(),
				ModuleConstants.COEFFICIENT_OF_FRICTION,
				DCMotor.getKrakenX60Foc(ModuleConstants.NUMBER_OF_DRIVE_MOTORS),
				KrakenX60DriveBuilder.GEAR_RATIO,
				KrakenX60DriveBuilder.SLIP_CURRENT,
				ModuleConstants.NUMBER_OF_DRIVE_MOTORS
			),
			swerve.getModules().getModulePositionsFromCenterMeters()
		);
	}


}
