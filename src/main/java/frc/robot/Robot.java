// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.poseestimator.GBPoseEstimator;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.PoseEstimatorConstants;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxDeviceID;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.poseestimation.PoseEstimator;
import frc.robot.structures.SuperStructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.swerveconstants.SwerveConstantsFactory;
import frc.robot.subsystems.swerve.swervestatehelpers.SwerveStateHelper;
import frc.utils.auto.AutonomousChooser;
import frc.utils.auto.PathPlannerUtils;
import frc.robot.vision.limelights.ILimelightFilterer;
import frc.robot.vision.limelights.LimelightFilterer;
import frc.robot.vision.limelights.LimelightFiltererConfig;
import frc.robot.vision.limelights.MultiLimelights;

import java.util.Optional;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();
	private static AutonomousChooser autonomousChooser;

	private final Swerve swerve;
	private final IPoseEstimator poseEstimator;
	private final SuperStructure superStructure;
	BrushlessSparkMAXMotor motor;

	public Robot() {
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(SwerveType.SWERVE),
			ModulesFactory.create(SwerveType.SWERVE),
			GyroFactory.create(SwerveType.SWERVE)
		);
//		this.poseEstimator = new PoseEstimator(swerve::setHeading, swerve.getConstants().kinematics());
		MultiLimelights multiLimelights = new MultiLimelights(new String[] {"limelight-front", "limelight-back"}, "Limelight/");
		ILimelightFilterer limelightFilterer = new LimelightFilterer(new LimelightFiltererConfig("LimelightFilterer", 1.2397), multiLimelights);
		this.poseEstimator = new GBPoseEstimator(
			"PoseEstimator/",
			limelightFilterer,
			swerve.getConstants().kinematics(),
			new SwerveDriveWheelPositions(new SwerveModulePosition[4]),
			new Rotation2d(),
			PoseEstimatorConstants.DEFAULT_ODOMETRY_STANDARD_DEVIATIONS,
			new Pose2d()
		);

		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
		swerve.setStateHelper(new SwerveStateHelper(() -> Optional.of(poseEstimator.getEstimatedPose()), Optional::empty, swerve));

		this.superStructure = new SuperStructure(swerve, poseEstimator);

		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(new SparkMaxDeviceID(3, CANSparkLowLevel.MotorType.kBrushless));

		motor = new BrushlessSparkMAXMotor("intake", sparkMaxWrapper, new SysIdRoutine.Config());

		buildPathPlannerForAuto();
		configureBindings();
	}

	private void buildPathPlannerForAuto() {
		// Register commands...
		PathPlannerUtils
			.registerCommand("Intake", new FunctionalCommand(() -> {}, () -> motor.setPower(0.6), (interrupted) -> motor.stop(), () -> false));
		PathPlannerUtils
			.registerCommand("Shoot", new FunctionalCommand(() -> {}, () -> motor.setPower(-0.6), (interrupted) -> motor.stop(), () -> false).withTimeout(3));
		swerve.configPathPlanner(poseEstimator::getEstimatedPose, poseEstimator::resetPose);
		autonomousChooser = new AutonomousChooser("Autonomous Chooser");
//		PathPlannerUtils.setTargetRotationOverride(() -> {
//			if (poseEstimator.getCurrentPose().getX() > 3)
//				return Optional.of(Rotation2d.fromDegrees(45));
//			else
//				return Optional.empty();
//		});
	}

	private void configureBindings() {
		JoysticksBindings.configureBindings(this);
	}


	public Command getAutonomousCommand() {
		return autonomousChooser.getChosenValue();
	}

	public SuperStructure getSuperStructure() {
		return superStructure;
	}

	public Swerve getSwerve() {
		return swerve;
	}

	public IPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

}
