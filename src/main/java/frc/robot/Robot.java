// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.RobotManager;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.factories.imu.IMUFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.vision.cameras.limelight.Limelight;
import frc.robot.vision.cameras.limelight.LimelightPipeline;
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

	private final Limelight limelightFront;
	private final Limelight limelightRight;
	private final Limelight limelightLeft;
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

		this.limelightFront = new Limelight(
				"limelight-front",
				"Vision",
				new Pose3d(
						new Translation3d(0.297, -0.143, 0.361),
						new Rotation3d(Math.toRadians(-0.18), Math.toRadians(27.38), Math.toRadians(-0.35))
				),
				LimelightPipeline.APRIL_TAG
		);
		this.limelightRight = new Limelight(
				"limelight-right",
				"Vision",
				new Pose3d(
						new Translation3d(-0.06, 0.367, 0.469),
						new Rotation3d(Math.toRadians(-177.78), Math.toRadians(20.64), Math.toRadians(-90.7))
				),
				LimelightPipeline.APRIL_TAG
		);
		this.limelightLeft = new Limelight(
				"limelight-left",
				"Vision",
				new Pose3d(
						new Translation3d(-0.125, -0.37, 0.481),
						new Rotation3d(Math.toRadians(-179.25), Math.toRadians(20.05), Math.toRadians(90.35))
				),
				LimelightPipeline.APRIL_TAG
		);

		BrakeStateManager.add(() -> swerve.getModules().setBrake(true), () -> swerve.getModules().setBrake(false));

		this.limelights = List.of(limelightFront, limelightRight, limelightLeft);
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

		swerve.setHeadingSupplier(() -> swerve.getIMUAbsoluteYaw().getValue());

		configureBrakeStateChooser();
	}

	public void updateSubsystems() {
		swerve.update();
	}

	public void periodic() {
		BusChain.refreshAll();

		updateSubsystems();
		getLimelights().forEach(Limelight::updateHardwareInputs);
		getLimelights().forEach(Limelight::updateMT1);

		BatteryUtil.logStatus();
		BusChain.logChainsStatuses();
		CommandScheduler.getInstance().run(); // Should be last
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
