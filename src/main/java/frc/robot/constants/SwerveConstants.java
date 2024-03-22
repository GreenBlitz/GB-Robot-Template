package frc.robot.constants;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleConstants;

public class SwerveConstants {

    public static final double MAX_SPEED = 1;
    public static final double WHEEL_DIAMETER = 1;

    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics();

    public static final SwerveModule SWERVE_MODULE_0 = new SwerveModule(
            0,
            new SwerveModuleConstants(
                    0,
                    0,
                    0,
                    Rotation2d.fromDegrees(0),
                    false,
                    false
            )
    );
    public static final int PIGEON_ID = 1;
    public static final SwerveModule SWERVE_MODULE_1 = new SwerveModule(
            0,
            new SwerveModuleConstants(
                    0,
                    0,
                    0,
                    Rotation2d.fromDegrees(0),
                    false,
                    false
            )
    );
    public static final SwerveModule SWERVE_MODULE_2 = new SwerveModule(
            0,
            new SwerveModuleConstants(
                    0,
                    0,
                    0
            )
    );
    public static final SwerveModule SWERVE_MODULE_3 = new SwerveModule(
            0,
            new SwerveModuleConstants(
                    0,
                    0,
                    0
            )
    );

    public static class DriveConstants {

        public static final double kS = 1;
        public static final double kV = 1;
        public static final double kA = 1;

        public static final double GEAR_RATIO = 0;

        public static final PIDController PID = new PIDController(0,0,0);

        public static final CurrentLimitsConfigs CURRENT_LIMIT = new CurrentLimitsConfigs();
        public static final OpenLoopRampsConfigs OPEN_LOOP_RAMPS_CONFIGS = new OpenLoopRampsConfigs();
        public static final ClosedLoopRampsConfigs CLOSED_LOOP_RAMPS_CONFIGS = new ClosedLoopRampsConfigs();
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;

        static {
            OPEN_LOOP_RAMPS_CONFIGS.DutyCycleOpenLoopRampPeriod = 1;
            OPEN_LOOP_RAMPS_CONFIGS.VoltageOpenLoopRampPeriod = 1;

            CLOSED_LOOP_RAMPS_CONFIGS.DutyCycleClosedLoopRampPeriod = 1;
            CLOSED_LOOP_RAMPS_CONFIGS.VoltageClosedLoopRampPeriod = 1;
        }

        static {
            CURRENT_LIMIT.SupplyCurrentLimit = 40;
            CURRENT_LIMIT.SupplyCurrentLimitEnable = true;
            CURRENT_LIMIT.SupplyCurrentThreshold = 0;
            CURRENT_LIMIT.SupplyTimeThreshold = 1;
        }
    }

    public static class AngleConstants {

        public static final double GEAR_RATIO = 0;

        public static final PIDController PID = new PIDController(0,0,0);

        public static final CurrentLimitsConfigs CURRENT_LIMIT = new CurrentLimitsConfigs();
        public static final NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Brake;
    }
}
