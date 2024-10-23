// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelConstants;
import frc.robot.subsystems.funnel.factory.FunnelFactory;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.factory.IntakeFactory;
import frc.robot.subsystems.elbow.Elbow;
import frc.robot.subsystems.elbow.ElbowConstants;
import frc.robot.subsystems.elbow.factory.ElbowFactory;
import frc.robot.subsystems.flywheel.FlyWheelConstants;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.factory.FlywheelFactory;
import frc.robot.subsystems.lifter.Lifter;
import frc.robot.subsystems.lifter.LifterConstants;
import frc.robot.subsystems.lifter.factory.LifterFactory;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.pivot.factory.PivotFactory;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.factory.RollerFactory;
import frc.robot.subsystems.solenoid.Solenoid;
import frc.robot.subsystems.solenoid.SolenoidConstants;
import frc.robot.subsystems.solenoid.factory.SolenoidFactory;
import frc.robot.poseestimator.GBPoseEstimator;
import frc.robot.poseestimator.PoseEstimatorConstants;
import frc.robot.poseestimator.VisionDenoiser;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.factories.gyro.GyroFactory;
import frc.robot.subsystems.swerve.factories.modules.ModulesFactory;
import frc.robot.subsystems.swerve.factories.swerveconstants.SwerveConstantsFactory;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristConstants;
import frc.robot.subsystems.wrist.factory.WristFactory;
import frc.robot.superstructure.RobotState;
import frc.robot.superstructure.StatesMotionPlanner;
import frc.robot.superstructure.Superstructure;
import frc.utils.auto.PathPlannerUtils;
import frc.utils.brakestate.BrakeStateManager;
import frc.robot.subsystems.swerve.swervestatehelpers.SwerveStateHelper;
import frc.robot.vision.limelights.LimeLightConstants;
import frc.robot.vision.limelights.LimelightFilterer;
import frc.robot.vision.limelights.LimelightFiltererConfig;
import frc.robot.vision.limelights.MultiLimelights;
import frc.utils.auto.AutonomousChooser;

import java.util.Arrays;
import java.util.Optional;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link RobotManager} periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class Robot {

	public static final RobotType ROBOT_TYPE = RobotType.determineRobotType();

	private AutonomousChooser autonomousChooser;

	private final GBPoseEstimator poseEstimator;
	private final LimelightFilterer limelightFilterer;
	private final MultiLimelights multiLimelights;

	private final Swerve swerve;
	private final Solenoid solenoid;
	private final Funnel funnel;
	private final Intake intake;
	private final Elbow elbow;
	private final Flywheel flywheel;
	private final Pivot pivot;
	private final Lifter lifter;
	private final Roller roller;
	private final Wrist wrist;

	private final Superstructure superstructure;
	private final StatesMotionPlanner statesMotionPlanner;

	public Robot() {
		this.swerve = new Swerve(
			SwerveConstantsFactory.create(SwerveType.SWERVE),
			ModulesFactory.create(SwerveType.SWERVE),
			GyroFactory.create(SwerveType.SWERVE)
		);
		this.solenoid = new Solenoid(SolenoidFactory.create(SolenoidConstants.LOG_PATH));
		this.intake = new Intake(IntakeFactory.create(IntakeConstants.LOG_PATH));
		this.flywheel = new Flywheel(FlywheelFactory.create(FlyWheelConstants.LOG_PATH));
		this.pivot = new Pivot(PivotFactory.create(PivotConstants.LOG_PATH));
		BrakeStateManager.add(() -> pivot.setBrake(true), () -> pivot.setBrake(false));
		this.elbow = new Elbow(ElbowFactory.create(ElbowConstants.LOG_PATH));
		BrakeStateManager.add(() -> elbow.setBrake(true), () -> elbow.setBrake(false));
		this.funnel = new Funnel(FunnelFactory.create(FunnelConstants.LOG_PATH));
		this.lifter = new Lifter(LifterFactory.create(LifterConstants.LOG_PATH));
		BrakeStateManager.add(() -> lifter.setBrake(true), () -> lifter.setBrake(false));
		this.roller = new Roller(RollerFactory.create(RollerConstants.LOG_PATH));
		BrakeStateManager.add(() -> roller.setBrake(true), () -> roller.setBrake(false));
		this.wrist = new Wrist(WristFactory.create(WristConstants.LOG_PATH));
		BrakeStateManager.add(() -> wrist.setBrake(true), () -> wrist.setBrake(false));

		this.multiLimelights = new MultiLimelights(LimeLightConstants.LIMELIGHT_NAMES, "limelightsHardware/");
		this.limelightFilterer = new LimelightFilterer(
			new LimelightFiltererConfig("limelightfilterer/", LimeLightConstants.DEFAULT_LIMELIGHT_FILTERS_TOLERANCES),
			multiLimelights
		);
		this.poseEstimator = new GBPoseEstimator(
			swerve::setHeading,
			"PoseEstimator/",
			limelightFilterer,
			swerve.getConstants().kinematics(),
			swerve.getModules().getWheelsPositions(0),
			swerve.getAbsoluteHeading(),
			PoseEstimatorConstants.DEFAULT_ODOMETRY_STANDARD_DEVIATIONS,
			new VisionDenoiser(PoseEstimatorConstants.LINEAR_FILTER_SAMPLES_FOR_EACH_VISION_CALCULATION)
		);
		limelightFilterer.setEstimatedPoseAtTimestampFunction(poseEstimator::getEstimatedPoseAtTimeStamp);

		swerve.setHeadingSupplier(() -> poseEstimator.getEstimatedPose().getRotation());
		swerve.setStateHelper(new SwerveStateHelper(() -> Optional.of(poseEstimator.getEstimatedPose()), Optional::empty, swerve));

		this.superstructure = new Superstructure("Superstructure/", this);
		this.statesMotionPlanner = new StatesMotionPlanner(superstructure);

		configPathPlanner();
		configureBindings();
	}

	public void periodic() {
		swerve.updateStatus();
//		poseEstimator.updateVision(limelightFilterer.getFilteredVisionObservations());
		poseEstimator.updateOdometry(Arrays.asList(swerve.getAllOdometryObservations()));
		superstructure.logStatus();
	}

	private void configPathPlanner() {
		// Register commands..
		PathPlannerUtils.registerCommand(
			RobotState.INTAKE_WITH_FLYWHEEL.name(),
			new SequentialCommandGroup(
				superstructure.enableChangeStateAutomatically(false),
				superstructure.setState(RobotState.INTAKE_WITH_FLYWHEEL).until(superstructure::isEnableChangeStateAutomatically)
			)
		);
		PathPlannerUtils.registerCommand(RobotState.PRE_SPEAKER.name(), superstructure.setState(RobotState.PRE_SPEAKER));
		PathPlannerUtils.registerCommand(
			RobotState.SPEAKER.name(),
			new SequentialCommandGroup(
				superstructure.enableChangeStateAutomatically(false),
				superstructure.setState(RobotState.SPEAKER_MANUAL_PIVOT).until(superstructure::isEnableChangeStateAutomatically)
			)
		);

		swerve.configPathPlanner(poseEstimator::getEstimatedPose, pose2d -> {});
		autonomousChooser = new AutonomousChooser("Autonomous Chooser");
	}

	private void configureBindings() {
		JoysticksBindings.configureBindings(this);
	}


	public Command getAutonomousCommand() {
		return autonomousChooser.getChosenValue();
	}

	public GBPoseEstimator getPoseEstimator() {
		return poseEstimator;
	}

	public Swerve getSwerve() {
		return swerve;
	}

	public Solenoid getSolenoid() {
		return solenoid;
	}

	public Funnel getFunnel() {
		return funnel;
	}

	public Intake getIntake() {
		return intake;
	}

	public Elbow getElbow() {
		return elbow;
	}

	public Flywheel getFlywheel() {
		return flywheel;
	}

	public Pivot getPivot() {
		return pivot;
	}

	public Lifter getLifter() {
		return lifter;
	}

	public Roller getRoller() {
		return roller;
	}

	public Wrist getWrist() {
		return wrist;
	}

	public Superstructure getSuperstructure() {
		return superstructure;
	}

	public StatesMotionPlanner getStatesMotionPlanner() {
		return statesMotionPlanner;
	}

}
