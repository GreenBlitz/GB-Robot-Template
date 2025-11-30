// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
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
import frc.utils.math.AngleTransform;
import frc.utils.math.FieldMath;
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

        Pose3d left = new Pose3d(
                new Translation3d(0.25009637774388915, -0.08377962196450461, 0.488331232141548),
                new Rotation3d(Units.Degrees.of(-8.554321498008184), Units.Degrees.of(-25.954173697872232), Units.Degrees.of(-20.586967248503125))
        );
        Pose3d right = new Pose3d(
                new Translation3d(0.2311823642106896, 0.13741969562525957, 0.49612288118246956),
                new Rotation3d(Units.Degrees.of(9.665914695321458), Units.Degrees.of(-26.155805464156003), Units.Degrees.of(21.681047144853935))
        );

        // minus the y and pitch

        LimelightHelpers.setCameraPose_RobotSpace("limelight-left", 0, 0, 0, 0, 0, 0);

        LimelightHelpers.setCameraPose_RobotSpace("limelight", 0, 0, 0, 0, 0, 0);


//		LimelightHelpers.setCameraPose_RobotSpace(
//				"limelight-left",
//				left.getX(), left.getY(), left.getZ(),
//				Math.toDegrees(left.getRotation().getX()),
//				Math.toDegrees(left.getRotation().getY()),
//				Math.toDegrees(left.getRotation().getZ())
//		);
//		LimelightHelpers.setCameraPose_RobotSpace(
//				"limelight",
//				right.getX(), right.getY(), right.getZ(),
//				Math.toDegrees(right.getRotation().getX()),
//				Math.toDegrees(right.getRotation().getY()),
//				Math.toDegrees(right.getRotation().getZ())
//		);
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

    /**
     * This function assumes:
     * - there is no difference in y between the tag and the robot
     * - the tag is at 180 degrees field relative
     * - you work with the newest tags map
     */
    public void logCameraPose(String PathPrefix, String cameraName, int tagID, double xRobotDistanceFromTag, double middleOfTagHeight) {
        Pose3d tagPoseFieldRelative = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded).getTagPose(tagID).get();
        Pose3d cameraPoseFieldRelative = LimelightHelpers.getBotPose3d_wpiBlue(cameraName);

        Pose2d robotPoseFieldRelative = new Pose2d(
            tagPoseFieldRelative.getX() - xRobotDistanceFromTag,
            tagPoseFieldRelative.getY(),
            FieldMath.transformAngle(tagPoseFieldRelative.getRotation().toRotation2d(), AngleTransform.INVERT)
        );

        // limelight is funny so we invert pitch
        Rotation3d endRot = new Rotation3d(
            cameraPoseFieldRelative.getRotation().getX(),
            -cameraPoseFieldRelative.getRotation().getY(),
            cameraPoseFieldRelative.getRotation().getZ() - robotPoseFieldRelative.getRotation().getRadians()
        );

        // limelight is funny so we invert y-axis
        Translation3d endTranslation = new Translation3d(
            cameraPoseFieldRelative.getX() - robotPoseFieldRelative.getX(),
            -(cameraPoseFieldRelative.getY() - robotPoseFieldRelative.getY()),
            cameraPoseFieldRelative.getZ() - tagPoseFieldRelative.getZ() + middleOfTagHeight
        );

        Logger.recordOutput("CameraCalibration/" + PathPrefix + "/tag/TagPoseFieldRelative", tagPoseFieldRelative);
        printRot3d("CameraCalibration/" + PathPrefix + "/tag/Rotation", tagPoseFieldRelative.getRotation());

        Logger.recordOutput("CameraCalibration/" + PathPrefix + "/cam/cameraPoseFieldRelative", cameraPoseFieldRelative);
        printRot3d("CameraCalibration/" + PathPrefix + "/cam/camRot", cameraPoseFieldRelative.getRotation());

        Logger.recordOutput("CameraCalibration/" + PathPrefix + "/robot/robotFieldRelative", robotPoseFieldRelative);

        printRot3d("CameraCalibration/" + PathPrefix + "/solution/endRot", endRot);
        Logger.recordOutput("CameraCalibration/" + PathPrefix + "/solution/endTranslation", endTranslation);
    }

    @Override
    public void robotPeriodic() {
        updateTimeRelatedData(); // Better to be first
        JoysticksBindings.updateChassisDriverInputs();
        robot.periodic();
        AlertManager.reportAlerts();

        int tagID = 18;
        double xRobotDistanceFromTag = (0.82 + 0.30833);
        double tagHeight = 0.08255 + 0.05;

        logCameraPose("Left", "limelight-left", tagID, xRobotDistanceFromTag, tagHeight);
        logCameraPose("Right", "limelight", tagID, xRobotDistanceFromTag, tagHeight);

        Logger.recordOutput("PoseCheck/Right", LimelightHelpers.getBotPose3d_wpiBlue("limelight"));
        Logger.recordOutput("PoseCheck/Left", LimelightHelpers.getBotPose3d_wpiBlue("limelight-left"));
    }

    public static void printRot3d(String path, Rotation3d rotation3d) {
        Logger.recordOutput(path + "-Radians", new double[]{rotation3d.getX(), rotation3d.getY(), rotation3d.getZ()});
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
