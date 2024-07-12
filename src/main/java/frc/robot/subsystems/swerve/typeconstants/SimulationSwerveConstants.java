package frc.robot.subsystems.swerve.typeconstants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.SwerveConstants;

public class SimulationSwerveConstants extends SwerveConstants {

    private final double MAX_SPEED_METERS_PER_SECOND = 5.052;
    private final Rotation2d MAX_ROTATIONAL_SPEED_PER_SECOND = Rotation2d.fromRadians(10);

    private final PIDConstants TRANSLATION_PID_CONSTANTS = new PIDConstants(6,0,0);
    private final PIDConstants ROTATION_PID_CONSTANTS = new PIDConstants(6,0,0);

    protected final PIDController TRANSLATION_PID_METERS_CONTROLLER = new PIDController(
            TRANSLATION_PID_CONSTANTS.kP,
            TRANSLATION_PID_CONSTANTS.kI,
            TRANSLATION_PID_CONSTANTS.kD
    );
    protected final PIDController ROTATION_PID_DEGREES_CONTROLLER = configRotationDegreesPIDController(ROTATION_PID_CONSTANTS);

    protected final HolonomicPathFollowerConfig HOLONOMIC_PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
            TRANSLATION_PID_CONSTANTS,
            ROTATION_PID_CONSTANTS,
            MAX_SPEED_METERS_PER_SECOND,
            DRIVE_RADIUS_METERS,
            REPLANNING_CONFIG
    );


    @Override
    protected double maxSpeedMetersPerSecond() {
        return MAX_SPEED_METERS_PER_SECOND;
    }

    @Override
    protected Rotation2d maxRotationSpeedPerSecond() {
        return MAX_ROTATIONAL_SPEED_PER_SECOND;
    }

    @Override
    protected PIDController translationMetersPIDController() {
        return TRANSLATION_PID_METERS_CONTROLLER;
    }

    @Override
    protected PIDController rotationDegreesPIDController() {
        return ROTATION_PID_DEGREES_CONTROLLER;
    }

    @Override
    protected HolonomicPathFollowerConfig holonomicPathFollowerConfig() {
        return HOLONOMIC_PATH_FOLLOWER_CONFIG;
    }

}
