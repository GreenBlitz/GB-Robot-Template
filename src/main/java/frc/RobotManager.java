// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousConstants;
import frc.utils.DriverStationUtil;
import frc.utils.LimelightHelpers;
import frc.utils.alerts.AlertManager;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.brakestate.BrakeStateManager;
import frc.utils.logger.LoggerFactory;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after creating this project, you must also update the build .gradle file in
 * the project.
 */
public class RobotManager extends LoggedRobot {

	private final Robot robot;
	private PathPlannerAutoWrapper autonomousCommand;
	private int roborioCycles;

	public RobotManager() {
		if (Robot.ROBOT_TYPE.isReplay()) {
			setUseTiming(false);
		}
		DriverStation.silenceJoystickConnectionWarning(true);
		LoggerFactory.initializeLogger();
		PathPlannerUtil.startPathfinder();
		PathPlannerUtil.setupPathPlannerLogging();

		this.roborioCycles = 0;
		this.robot = new Robot();

		createAutoReadyForConstructionChooser();
		JoysticksBindings.configureBindings(robot);

		Threads.setCurrentThreadPriority(true, 10);
	}

	@Override
	public void disabledInit() {
		if (!DriverStationUtil.isMatch()) {
			BrakeStateManager.coast();
		}
	}

	@Override
	public void disabledExit() {
		if (!DriverStationUtil.isMatch()) {
			BrakeStateManager.brake();
		}
	}

	@Override
	public void autonomousInit() {
		// robot.getRobotCommander().getSuperstructure().setIsSubsystemRunningIndependently(true);
		// robot.getSwerve().getCommandsBuilder().setIsSubsystemRunningIndependently(true);

		if (autonomousCommand == null) {
			this.autonomousCommand = robot.getAutonomousCommand();
		}
		autonomousCommand.schedule();
	}

	@Override
	public void autonomousExit() {
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
		// robot.getRobotCommander().getSuperstructure().setIsSubsystemRunningIndependently(false);
		// robot.getSwerve().getCommandsBuilder().setIsSubsystemRunningIndependently(false);
	}

	TestInputsAutoLogged testInputs = new TestInputsAutoLogged();

	@Override
	public void robotPeriodic() {
		updateTimeRelatedData(); // Better to be first
		JoysticksBindings.updateChassisDriverInputs();
		robot.periodic();
		AlertManager.reportAlerts();

		// todo calibrate - tagID [ ] , camera name - [X], expected pose - [X] , xRobotDistanceFromTag [ ] , tagHeight [ ]

		String cameraName = "limelight-left";
		int tagID = 13;
		double xRobotDistanceFromTag = 1;
		double tagHeight = 0.2;

		LimelightHelpers.setCameraPose_RobotSpace(cameraName, 0, 0, 0, 0, 0, 0);

		Pose3d tagPoseFieldRelative = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(tagID).get();

		testInputs.cameraPoseFieldRelative = LimelightHelpers.getBotPose3d_wpiBlue(cameraName);
		Logger.processInputs("Test/InputsForReplay", testInputs);

		Pose3d cameraPoseFieldRelative = testInputs.cameraPoseFieldRelative;

		// tag - robotToTagFieldRelative = robot
		Pose3d robotToTagFieldRelative = new Pose3d(
			xRobotDistanceFromTag * Math.cos(tagPoseFieldRelative.getRotation().getZ()),
			xRobotDistanceFromTag * Math.sin(tagPoseFieldRelative.getRotation().getZ()),
			tagPoseFieldRelative.getZ() - tagHeight,
			new Rotation3d(0, 0, 0)
		);

		Pose3d robotPoseFieldRelative = new Pose3d(
			tagPoseFieldRelative.getX() - robotToTagFieldRelative.getX(),
			tagPoseFieldRelative.getY() - robotToTagFieldRelative.getY(),
			tagPoseFieldRelative.getZ() - robotToTagFieldRelative.getZ(),
			tagPoseFieldRelative.getRotation().minus(robotToTagFieldRelative.getRotation())
		);

		// when robot looks at tag yaw 0 its yaw is 180 (so we invert yaw)
		Rotation3d endRot = new Rotation3d(
			cameraPoseFieldRelative.getRotation().getX(),
			cameraPoseFieldRelative.getRotation().getY(),
			cameraPoseFieldRelative.getRotation().getZ() - (Math.PI + tagPoseFieldRelative.getRotation().getZ())
		);

		double distanceCameraToRobotXY = cameraPoseFieldRelative.toPose2d()
			.getTranslation()
			.getDistance(robotPoseFieldRelative.toPose2d().getTranslation());
		Translation3d endTranslation = new Translation3d(
			distanceCameraToRobotXY * Math.cos(robotPoseFieldRelative.getRotation().getZ()),
			distanceCameraToRobotXY * Math.sin(robotPoseFieldRelative.getRotation().getZ()),
			cameraPoseFieldRelative.getZ() - robotPoseFieldRelative.getZ()
		);

		Pose3d expected = new Pose3d(
			new Translation3d(0.215, -0.11, 0.508),
			new Rotation3d(Units.Degrees.of(-8.06180374425555), Units.Degrees.of(-27.07784559039065), Units.Degrees.of(-22.52372569716833))
		);

		Logger.recordOutput("Test/tag/TagPoseFieldRelative", tagPoseFieldRelative);
		printRot3d("Test/tag/Rotation", tagPoseFieldRelative.getRotation());

		Logger.recordOutput("Test/cam/cameraPoseFieldRelative", cameraPoseFieldRelative);
		printRot3d("Test/cam/camRot", cameraPoseFieldRelative.getRotation());

		Logger.recordOutput("Test/robot/robotToTagFieldRelative", robotToTagFieldRelative);
		Logger.recordOutput("Test/robot/robotFieldRelative", robotPoseFieldRelative);

		printRot3d("Test/solution/endRot", endRot);
		Logger.recordOutput("Test/solution/endTranslation", endTranslation);

		Logger.recordOutput("Test/expected/expected", expected.getTranslation());
		printRot3d("Test/expected/expectedRot", expected.getRotation());
	}

	public static void printRot3d(String path, Rotation3d rotation3d) {
		Logger.recordOutput(path + "-Radians", new double[] {rotation3d.getX(), rotation3d.getY(), rotation3d.getZ()});
	}

	private void createAutoReadyForConstructionChooser() {
		SendableChooser<Boolean> autoReadyForConstructionSendableChooser = new SendableChooser<>();
		autoReadyForConstructionSendableChooser.setDefaultOption("false", false);
		autoReadyForConstructionSendableChooser.addOption("true", true);
		autoReadyForConstructionSendableChooser.onChange(isReady -> {
			if (isReady) {
				this.autonomousCommand = robot.getAutonomousCommand();
				BrakeStateManager.brake();
			} else {
				BrakeStateManager.coast();
			}
			Logger.recordOutput(AutonomousConstants.LOG_PATH_PREFIX + "/ReadyToConstruct", isReady);
		});
		SmartDashboard.putData("AutoReadyForConstruction", autoReadyForConstructionSendableChooser);
	}

	private void updateTimeRelatedData() {
		roborioCycles++;
		Logger.recordOutput("RoborioCycles", roborioCycles);
		TimeUtil.updateCycleTime(roborioCycles);
	}

}
