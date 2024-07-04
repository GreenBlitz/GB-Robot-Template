package frc.robot.subsystems.swerve;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.constants.LogPathsConstants;
import frc.robot.constants.MathConstants;

public class SwerveConstants {
    //todo - swerve const as object that depends on sim or real or robot, calibration const (all const that needs calibration (maybe))

    public static final String SWERVE_LOG_PATH = LogPathsConstants.SUBSYSTEM_LOG_PATH + "Swerve/";
    protected static final String SWERVE_STATE_LOG_PATH = SWERVE_LOG_PATH + "Current State/";
    protected static final String SWERVE_VELOCITY_LOG_PATH = SWERVE_LOG_PATH + "Velocity/";


    protected static final Rotation2d WHEEL_RADIUS_CALIBRATION_VELOCITY = Rotation2d.fromRotations(0.5);
    protected static final double STEER_SYSID_CALIBRATION_VOLTAGE_STEP = 1;
    protected static final double STEER_SYSID_CALIBRATION_RAMP_RATE = 0.5;
    protected static final double DRIVE_SYSID_CALIBRATION_VOLTAGE_STEP = 2;
    protected static final double DRIVE_SYSID_CALIBRATION_RAMP_RATE = 0.5;


    public static final double AIM_ASSIST_MAGNITUDE_FACTOR = 3; // todo - calibrate
    public static final double SLOW_DRIVE_MODE_FACTOR = 0.5;

    protected static final double DRIVE_NEUTRAL_DEADBAND = 0.2;
    protected static final Rotation2d ROTATION_NEUTRAL_DEADBAND = Rotation2d.fromRadians(0.2);


    //todo - actual max in sim maybe in real will be 5.27. sim need moi calibration
    public static final double MAX_SPEED_METERS_PER_SECOND = 5.033;//todo - calibrate for real
    public static final Rotation2d MAX_ROTATIONAL_SPEED_PER_SECOND = Rotation2d.fromRadians(10);//todo - calibrate for real


    private static final double MODULE_X_DISTANCE_FROM_CENTER = 0.27833;
    private static final double MODULE_Y_DISTANCE_FROM_CENTER = 0.34733;
    public static final double DRIVE_RADIUS_METERS = Math.hypot(MODULE_X_DISTANCE_FROM_CENTER, MODULE_Y_DISTANCE_FROM_CENTER);

    public static final Translation2d FRONT_LEFT_TRANSLATION2D = new Translation2d(
            MODULE_X_DISTANCE_FROM_CENTER,
            MODULE_Y_DISTANCE_FROM_CENTER
    );
    public static final Translation2d FRONT_RIGHT_TRANSLATION2D = new Translation2d(
            MODULE_X_DISTANCE_FROM_CENTER,
            -MODULE_Y_DISTANCE_FROM_CENTER
    );
    public static final Translation2d BACK_LEFT_TRANSLATION2D = new Translation2d(
            -MODULE_X_DISTANCE_FROM_CENTER,
            MODULE_Y_DISTANCE_FROM_CENTER
    );
    public static final Translation2d BACK_RIGHT_TRANSLATION2D = new Translation2d(
            -MODULE_X_DISTANCE_FROM_CENTER,
            -MODULE_Y_DISTANCE_FROM_CENTER
    );

    public static final Translation2d[] LOCATIONS = {
            FRONT_LEFT_TRANSLATION2D,
            FRONT_RIGHT_TRANSLATION2D,
            BACK_LEFT_TRANSLATION2D,
            BACK_RIGHT_TRANSLATION2D
    };
    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(LOCATIONS);


    protected static final PIDController ROTATION_PID_DEGREES_CONTROLLER = new PIDController(5.5, 0, 0);//todo - calibrate
    static {
        ROTATION_PID_DEGREES_CONTROLLER.enableContinuousInput(
                -MathConstants.HALF_CIRCLE.getDegrees(),
                MathConstants.HALF_CIRCLE.getDegrees()
        );
    }

    protected static final PIDController TRANSLATION_PID_CONTROLLER = new PIDController(6, 0, 0);//todo - calibrate


    private static final PIDConstants AUTO_TRANSLATION_PID_CONSTANTS = new PIDConstants(6, 0, 0);//todo - calibrate
    private static final PIDConstants AUTO_ROTATION_PID_CONSTANTS = new PIDConstants(4, 0, 0);//todo - calibrate
    private static final ReplanningConfig REPLANNING_CONFIG = new ReplanningConfig(true, true);
    public static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
            AUTO_TRANSLATION_PID_CONSTANTS,
            AUTO_ROTATION_PID_CONSTANTS,
            MAX_SPEED_METERS_PER_SECOND,
            DRIVE_RADIUS_METERS,
            REPLANNING_CONFIG
    );

    public static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(
            2.5, 2.5, 4, 4
    );
    protected static final double CLOSE_TO_TARGET_POSITION_DEADBAND_METERS = 0.5;

}
