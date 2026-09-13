// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorConstants;
import frc.robot.poseestimator.WPILibPoseEstimator.WPILibPoseEstimatorWrapper;
import frc.robot.subsystems.FlywheelConstants;
import frc.robot.subsystems.roller.TalonFXRollerBuilder;
import frc.robot.subsystems.roller.VelocityRoller;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.factories.imu.IMUFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.vision.cameras.limelight.Limelight;
import frc.robot.vision.cameras.limelight.LimelightFilters;
import frc.robot.vision.cameras.limelight.LimelightStdDevCalculations;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.battery.BatteryUtil;
import frc.robot.hardware.interfaces.IIMU;
import frc.utils.brakestate.BrakeMode;
import frc.utils.brakestate.BrakeStateManager;

import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType(false);

	private final Swerve swerve;
	public final VelocityRoller flywheel;
	private final IPoseEstimator poseEstimator;
	private final List<Limelight> limelights;

	public Robot() {
		BatteryUtil.scheduleLimiter();

		IIMU imu = IMUFactory.createIMU(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve");
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			ModulesFactory.create(RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Swerve"),
			imu,
			IMUFactory.createSignals(imu)
		);

		this.flywheel = TalonFXRollerBuilder.buildVelocityRoller("flywheel", new Phoenix6DeviceID(10, BusChain.ROBORIO),buildConfig().Slot0,buildConfig().Slot0,buildFollowerConfig(),new SysIdRoutine.Config(
				Units.Volts.of(1).per(Units.Second),
				Units.Volts.of(7),
				null,
				state -> SignalLogger.writeString( "flywheel/state", state.toString())
		),FlywheelConstants.CURRENT_LIMIT,buildConfig().Feedback,FlywheelConstants.MOMENT_OF_INERTIA,false,true);

		BrakeStateManager.add(() -> swerve.getModules().setBrake(true), () -> swerve.getModules().setBrake(false));
		this.poseEstimator = new WPILibPoseEstimatorWrapper(
			WPILibPoseEstimatorConstants.WPILIB_POSEESTIMATOR_LOGPATH,
			swerve.getKinematics(),
			swerve.getModules().getWheelPositions(0),
			swerve.getModules().getCurrentStates(),
			swerve.getOrientationFromIMU(),
			swerve.getIMUAccelerationG(),
			swerve.getIMUAbsoluteYaw().getTimestamp()
		);

		this.limelights = List.of();
		limelights.forEach(
			limelight -> limelight.setMT1StdDevsCalculation(
				LimelightStdDevCalculations.getMT1StdDevsCalculation(
					limelight,
					RobotConstants.DEFAULT_TAG_DISTANCE_FACTORS,
					RobotConstants.DEFAULT_STD_DEV_FACTORS,
					RobotConstants.DEFAULT_VISIBLE_TAGS_EXPONENTS,
					RobotConstants.DEFAULT_STD_DEV_ADDITIONS
				)
			)
		);
		limelights.forEach(
			limelight -> limelight.setMT1PoseFilter(
				LimelightFilters.megaTag1Filter(
					limelight,
					timestamp -> poseEstimator.getEstimatedPoseAtTimestamp(timestamp).map(Pose2d::getRotation),
					poseEstimator::isIMUOffsetCalibrated,
					LimelightFilters.DEFAULT_IN_FIELD_TOLERANCE_METERS,
					LimelightFilters.DEFAULT_YAW_AT_ANGLE_TOLERANCE
				)
			)
		);

		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());

		configureBrakeStateChooser();
	}

	public void updateSubsystems() {
		swerve.update();
		flywheel.update();
	}

	public static TalonFXFollowerConfig buildFollowerConfig() {
		TalonFXFollowerConfig followerConfig = new TalonFXFollowerConfig();

		followerConfig.motorConfig.MotorOutput.Inverted = FlywheelConstants.IS_FOLLOWER_INVERTED;
		followerConfig.motorConfig.Feedback.SensorToMechanismRatio = FlywheelConstants.SENSOR_TO_MECHANISM_RATIO_FOLLOWER;

		followerConfig.motorConfig.CurrentLimits.StatorCurrentLimit = FlywheelConstants.CURRENT_LIMIT;
		followerConfig.motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		followerConfig.motorConfig.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.CURRENT_LIMIT;
		followerConfig.motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

		followerConfig.followerIDs = new TalonFXFollowerConfig.TalonFXFollowerID[] {
				new TalonFXFollowerConfig.TalonFXFollowerID("flyWheelFollower", new Phoenix6DeviceID(11, BusChain.ROBORIO), MotorAlignmentValue.Opposed)};

		return followerConfig;
	}

	public void periodic() {
		BusChain.refreshAll();

		updateSubsystems();
		poseEstimator.updateOdometry(swerve.getAllOdometryData());

		getLimelights().forEach(Limelight::updateHardwareInputs);
		getLimelights().forEach(Limelight::updateMT1);
		getLimelights().forEach(limelight -> limelight.getIndependentRobotPose().ifPresent(poseEstimator::updateVision));

		poseEstimator.log();

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
	}

	public static TalonFXConfiguration buildConfig() {
		TalonFXConfiguration configuration = new TalonFXConfiguration();

		configuration.Feedback.SensorToMechanismRatio = FlywheelConstants.SENSOR_TO_MECHANISM_RATIO_MASTER;

		if (Robot.ROBOT_TYPE.equals(RobotType.REAL)) {
			configuration.Slot0.kP = FlywheelConstants.kP;
			configuration.Slot0.kI = FlywheelConstants.kI;
			configuration.Slot0.kD = FlywheelConstants.kD;
			configuration.Slot0.kV = FlywheelConstants.kV;
			configuration.Slot0.kA = FlywheelConstants.kA;
			configuration.Slot0.kS = FlywheelConstants.kS;
		} else {
			configuration.Slot0.kP = FlywheelConstants.kP_SIM;
			configuration.Slot0.kI = FlywheelConstants.kI_SIM;
			configuration.Slot0.kD = FlywheelConstants.kD_SIM;
			configuration.Slot0.kV = FlywheelConstants.kV_SIM;
			configuration.Slot0.kA = FlywheelConstants.kA_SIM;
			configuration.Slot0.kS = FlywheelConstants.kS_SIM;
		}

		return configuration;
	}

	public IPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

	public Swerve getSwerve() {
		return swerve;
	}

	public List<Limelight> getLimelights() {
		return limelights;
	}

	public PathPlannerAutoWrapper getAutonomousCommand() {
		return new PathPlannerAutoWrapper();
	}

	private void configureBrakeStateChooser() {
		SendableChooser<BrakeMode> brakeStateChooser = new SendableChooser<>();
		brakeStateChooser.setDefaultOption("Brake", BrakeMode.BRAKE);
		brakeStateChooser.addOption("Coast", BrakeMode.COAST);
		SmartDashboard.putData("BrakeState", brakeStateChooser);
		brakeStateChooser.onChange(BrakeStateManager::setBrakeMode);
	}

}
