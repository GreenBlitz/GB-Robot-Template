package frc.robot.subsystems.swerve.falconswerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Phoenix6Constants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.utils.RobotTypeUtils;


import java.util.Optional;

public class FalconSwerveConstants extends SwerveConstants {
    public static final double
            MAX_SPEED_METERS_PER_SECOND = 4.25,
            MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND = 12.03;

    private static final double
            MAX_ROTATION_VELOCITY = 720,
            MAX_ROTATION_ACCELERATION = 720;

    private static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
            MAX_ROTATION_VELOCITY,
            MAX_ROTATION_ACCELERATION
    );

    private static final double
            MODULE_X_DISTANCE_FROM_CENTER = 0.6457 / 2,
            MODULE_Y_DISTANCE_FROM_CENTER = 0.5357 / 2;
    private static final Translation2d[] LOCATIONS = {
            new Translation2d(MODULE_X_DISTANCE_FROM_CENTER, MODULE_Y_DISTANCE_FROM_CENTER),
            new Translation2d(MODULE_X_DISTANCE_FROM_CENTER, -MODULE_Y_DISTANCE_FROM_CENTER),
            new Translation2d(-MODULE_X_DISTANCE_FROM_CENTER, MODULE_Y_DISTANCE_FROM_CENTER),
            new Translation2d(-MODULE_X_DISTANCE_FROM_CENTER, -MODULE_Y_DISTANCE_FROM_CENTER)
    };
    private static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(LOCATIONS);

    private static final Optional<SwerveModuleIO[]> MODULES_IO = ofReplayable(() -> new SwerveModuleIO[]{
            new FalconSwerveModule(FalconSwerveModuleConstants.FRONT_LEFT_SWERVE_MODULE_CONSTANTS, "FrontLeft"),
            new FalconSwerveModule(FalconSwerveModuleConstants.FRONT_RIGHT_SWERVE_MODULE_CONSTANTS, "FrontRight"),
            new FalconSwerveModule(FalconSwerveModuleConstants.BACK_LEFT_SWERVE_MODULE_CONSTANTS, "BackLeft"),
            new FalconSwerveModule(FalconSwerveModuleConstants.BACK_RIGHT_SWERVE_MODULE_CONSTANTS, "BackRight")
    });

    private static final PIDConstants
            TRANSLATION_PID_CONSTANTS = new PIDConstants(5, 0, 0),
            PROFILED_ROTATION_PID_CONSTANTS = new PIDConstants(6, 0, 0),
            AUTO_TRANSLATION_PID_CONSTANTS = new PIDConstants(5, 0, 0),
            AUTO_ROTATION_PID_CONSTANTS = new PIDConstants(3, 0, 0);

    private static final ProfiledPIDController PROFILED_ROTATION_PID_CONTROLLER = new ProfiledPIDController(
            PROFILED_ROTATION_PID_CONSTANTS.kP,
            PROFILED_ROTATION_PID_CONSTANTS.kI,
            PROFILED_ROTATION_PID_CONSTANTS.kD,
            ROTATION_CONSTRAINTS
    );
    private static final PIDController TRANSLATION_PID_CONTROLLER = new PIDController(
            TRANSLATION_PID_CONSTANTS.kP,
            TRANSLATION_PID_CONSTANTS.kI,
            TRANSLATION_PID_CONSTANTS.kD
    );

    private static final int PIGEON_ID = 0;
    private static final Rotation3d GYRO_MOUNT_POSITION = new Rotation3d(
            Units.degreesToRadians(0),
            Units.degreesToRadians(0),
            Units.degreesToRadians(0)
    );
    static final Optional<Pigeon2> GYRO = ofReplayable(() -> new Pigeon2(PIGEON_ID, Phoenix6Constants.CANIVORE_NAME));

    private static final double DRIVE_RADIUS_METERS = Math.hypot(
            MODULE_X_DISTANCE_FROM_CENTER, MODULE_Y_DISTANCE_FROM_CENTER
    );
    private static final ReplanningConfig REPLANNING_CONFIG = new ReplanningConfig(true, true);
    private static final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
            AUTO_TRANSLATION_PID_CONSTANTS,
            AUTO_ROTATION_PID_CONSTANTS,
            MAX_SPEED_METERS_PER_SECOND,
            DRIVE_RADIUS_METERS,
            REPLANNING_CONFIG
    );

    static StatusSignal<Double>
            YAW_SIGNAL = null,
            PITCH_SIGNAL = null,
            X_ACCELERATION_SIGNAL = null,
            Y_ACCELERATION_SIGNAL = null,
            Z_ACCELERATION_SIGNAL = null;

    static {
        if (!RobotTypeUtils.isReplay())
            configureGyro();
    }

    private static void configureGyro() {
        final Pigeon2 gyro = GYRO.get();
        final Pigeon2Configuration config = new Pigeon2Configuration();

        config.MountPose.MountPoseRoll = Units.radiansToDegrees(GYRO_MOUNT_POSITION.getX());
        config.MountPose.MountPosePitch = Units.radiansToDegrees(GYRO_MOUNT_POSITION.getY());
        config.MountPose.MountPoseYaw = Units.radiansToDegrees(GYRO_MOUNT_POSITION.getZ());

        gyro.getConfigurator().apply(config);

        YAW_SIGNAL = gyro.getYaw();
        PITCH_SIGNAL = gyro.getPitch();
        X_ACCELERATION_SIGNAL = gyro.getAccelerationX();
        Y_ACCELERATION_SIGNAL = gyro.getAccelerationY();
        Z_ACCELERATION_SIGNAL = gyro.getAccelerationZ();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                X_ACCELERATION_SIGNAL,
                Y_ACCELERATION_SIGNAL,
                Z_ACCELERATION_SIGNAL
        );

        PITCH_SIGNAL.setUpdateFrequency(100);

        YAW_SIGNAL.setUpdateFrequency(PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ);//need to be 250

        gyro.optimizeBusUtilization();
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return KINEMATICS;
    }

    @Override
    public Optional<Pigeon2> getPigeon() {
        return GYRO;
    }

    @Override
    protected Optional<SwerveModuleIO[]> getModulesIO() {
        return MODULES_IO;
    }

    @Override
    protected HolonomicPathFollowerConfig getPathFollowerConfig() {
        return HOLONOMIC_PATH_FOLLOWER_CONFIG;
    }

    @Override
    protected double getMaxSpeedMetersPerSecond() {
        return MAX_SPEED_METERS_PER_SECOND;
    }

    @Override
    protected double getMaxRotationalSpeedRadiansPerSecond() {
        return MAX_ROTATIONAL_SPEED_RADIANS_PER_SECOND;
    }

    @Override
    protected ProfiledPIDController getProfiledRotationController() {
        return PROFILED_ROTATION_PID_CONTROLLER;
    }

    @Override
    protected PIDController getTranslationsController() {
        return TRANSLATION_PID_CONTROLLER;
    }
}
