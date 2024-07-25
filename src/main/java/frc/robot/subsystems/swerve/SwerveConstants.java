package frc.robot.subsystems.swerve;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.constants.MathConstants;
import frc.robot.subsystems.swerve.gyro.SwerveGyroConstants;
import frc.robot.subsystems.swerve.modules.ModuleConstants;

public record SwerveConstants(
        double velocityAt12VoltsMetersPerSecond,
        Rotation2d maxRotationalVelocityPerSecond,
        PIDController translationMetersPIDController,
        PIDController rotationDegreesPIDController,
        HolonomicPathFollowerConfig holonomicPathFollowerConfig,
        ModuleConstants[] moduleConstants,
        SwerveGyroConstants gyroConstants
) {

    public SwerveConstants(
            double velocityAt12VoltsMetersPerSecond,
            Rotation2d maxRotationalVelocityPerSecond,
            PIDConstants translationMetersPIDConstants,
            PIDConstants rotationDegreesPIDConstants,
            ModuleConstants[] moduleConstants,
            SwerveGyroConstants gyroConstants
    ) {
        this(
                velocityAt12VoltsMetersPerSecond,
                maxRotationalVelocityPerSecond,
                new PIDController(
                        translationMetersPIDConstants.kP,
                        translationMetersPIDConstants.kI,
                        translationMetersPIDConstants.kD
                ),
                new PIDController(
                        rotationDegreesPIDConstants.kP,
                        rotationDegreesPIDConstants.kI,
                        rotationDegreesPIDConstants.kD
                ),
                new HolonomicPathFollowerConfig(
                        translationMetersPIDConstants,
                        rotationDegreesPIDConstants,
                        velocityAt12VoltsMetersPerSecond,
                        DRIVE_RADIUS_METERS,
                        REPLANNING_CONFIG
                ),
                moduleConstants,
                gyroConstants
        );

        this.rotationDegreesPIDController.enableContinuousInput(
                -MathConstants.HALF_CIRCLE.getDegrees(),
                MathConstants.HALF_CIRCLE.getDegrees()
        );
    }

    public static final String SWERVE_LOG_PATH = "Subsystems/Swerve/";
    static final String SWERVE_STATE_LOG_PATH = SWERVE_LOG_PATH + "Current State/";
    static final String SWERVE_VELOCITY_LOG_PATH = SWERVE_LOG_PATH + "Velocity/";

    static final Rotation2d WHEEL_RADIUS_CALIBRATION_VELOCITY = Rotation2d.fromRotations(0.5);
    static final double STEER_SYSID_CALIBRATION_VOLTAGE_STEP = 1;
    static final double STEER_SYSID_CALIBRATION_RAMP_RATE = 0.5;
    static final double DRIVE_SYSID_CALIBRATION_VOLTAGE_STEP = 2;
    static final double DRIVE_SYSID_CALIBRATION_RAMP_RATE = 0.5;

    public static final double AIM_ASSIST_MAGNITUDE_FACTOR = 4;

    static final double DRIVE_NEUTRAL_DEADBAND = 0.2;
    static final Rotation2d ROTATION_NEUTRAL_DEADBAND = Rotation2d.fromRadians(0.2);

    private static final double MODULE_X_DISTANCE_FROM_CENTER = 0.27833;
    private static final double MODULE_Y_DISTANCE_FROM_CENTER = 0.34733;
    static final double DRIVE_RADIUS_METERS = Math.hypot(MODULE_X_DISTANCE_FROM_CENTER, MODULE_Y_DISTANCE_FROM_CENTER);
    private static final Translation2d FRONT_LEFT_TRANSLATION2D = new Translation2d(
            MODULE_X_DISTANCE_FROM_CENTER,
            MODULE_Y_DISTANCE_FROM_CENTER
    );
    private static final Translation2d FRONT_RIGHT_TRANSLATION2D = new Translation2d(
            MODULE_X_DISTANCE_FROM_CENTER,
            -MODULE_Y_DISTANCE_FROM_CENTER
    );
    private static final Translation2d BACK_LEFT_TRANSLATION2D = new Translation2d(
            -MODULE_X_DISTANCE_FROM_CENTER,
            MODULE_Y_DISTANCE_FROM_CENTER
    );
    private static final Translation2d BACK_RIGHT_TRANSLATION2D = new Translation2d(
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

    private static final ReplanningConfig REPLANNING_CONFIG = new ReplanningConfig(true, true);
    static final PathConstraints REAL_TIME_CONSTRAINTS = new PathConstraints(2.5, 2.5, 4, 4);
    static final double CLOSE_TO_TARGET_POSITION_DEADBAND_METERS = 0.5;

}
