package frc.robot.subsystems;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmSimulationConstants;
import frc.robot.subsystems.arm.DynamicMotionMagicArm;
import frc.robot.subsystems.arm.TalonFXArmBuilder;

public class HoodConstants {

    public static final boolean IS_INVERTED = true;
    public static final boolean IS_CONTINUOUS_WRAP = false;

    public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs();
    public static final Slot0Configs REAL_SLOT = new Slot0Configs();
    public static final Slot0Configs SIMULATION_SLOT = new Slot0Configs();

    static {
        FEEDBACK_CONFIGS.RotorToSensorRatio = 1;
        FEEDBACK_CONFIGS.SensorToMechanismRatio = 30.0;

        REAL_SLOT.kP = 600;
        REAL_SLOT.kI = 0;
        REAL_SLOT.kD = 0;
        REAL_SLOT.kS = 0.3;
        REAL_SLOT.kG = 0;
        REAL_SLOT.kV = 0;
        REAL_SLOT.kA = 0;
        REAL_SLOT.GravityType = GravityTypeValue.Arm_Cosine;
        REAL_SLOT.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

        SIMULATION_SLOT.kP = 20;
        SIMULATION_SLOT.kI = 0;
        SIMULATION_SLOT.kD = 0;
        SIMULATION_SLOT.kG = 0;
        SIMULATION_SLOT.kS = 0;
        SIMULATION_SLOT.GravityType = GravityTypeValue.Arm_Cosine;
    }

    public static final Rotation2d MAXIMUM_POSITION = Rotation2d.fromDegrees(55);
    public static final Rotation2d MINIMUM_POSITION = Rotation2d.fromDegrees(27.5);

    public static final Rotation2d FORWARD_SOFTWARE_LIMIT = Rotation2d.fromDegrees(MAXIMUM_POSITION.getDegrees() - 0.1);
    public static final Rotation2d BACKWARD_SOFTWARE_LIMIT = Rotation2d.fromDegrees(MINIMUM_POSITION.getDegrees() + 0.1);
    public static final double CURRENT_LIMIT = 40;

    public static final double HOOD_LENGTH_METERS = 0.3;
    public static final double ARBITRARY_FEEDFORWARD = 0;
    public static final double MOMENT_OF_INERTIA = 0.001;
    public static final SysIdRoutine.Config SYSID_ROUTINE_CONFIG = new SysIdRoutine.Config();

    public static Arm createArm() {
        ArmSimulationConstants hoodSimulationConstants = new ArmSimulationConstants(
                MAXIMUM_POSITION,
                MINIMUM_POSITION,
                MINIMUM_POSITION,
                MOMENT_OF_INERTIA,
                HOOD_LENGTH_METERS
        );
        return TalonFXArmBuilder.buildArm(
                RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Arm",
                IDs.TalonFXIDs.HOOD,
                IS_INVERTED,
                IS_CONTINUOUS_WRAP,
                new TalonFXFollowerConfig(),
                SYSID_ROUTINE_CONFIG,
                FEEDBACK_CONFIGS,
                REAL_SLOT,
                SIMULATION_SLOT,
                CURRENT_LIMIT,
                RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
                ARBITRARY_FEEDFORWARD,
                FORWARD_SOFTWARE_LIMIT,
                BACKWARD_SOFTWARE_LIMIT,
                hoodSimulationConstants
        );
    }

    public static Arm createMotionMagicArm() {
        ArmSimulationConstants hoodSimulationConstants = new ArmSimulationConstants(
                MAXIMUM_POSITION,
                MINIMUM_POSITION,
                MINIMUM_POSITION,
                MOMENT_OF_INERTIA,
                HOOD_LENGTH_METERS
        );
        return TalonFXArmBuilder.buildMotionMagicArm(
                RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/MotionMagicArm",
                IDs.TalonFXIDs.NOT_HOOD1,
                IS_INVERTED,
                IS_CONTINUOUS_WRAP,
                new TalonFXFollowerConfig(),
                SYSID_ROUTINE_CONFIG,
                FEEDBACK_CONFIGS,
                REAL_SLOT,
                SIMULATION_SLOT,
                CURRENT_LIMIT,
                RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
                ARBITRARY_FEEDFORWARD,
                FORWARD_SOFTWARE_LIMIT,
                BACKWARD_SOFTWARE_LIMIT,
                hoodSimulationConstants,
                Rotation2d.fromDegrees(5),
                Rotation2d.fromDegrees(5)
        );
    }public static DynamicMotionMagicArm createDynamicMotionMagicArm() {
        ArmSimulationConstants hoodSimulationConstants = new ArmSimulationConstants(
                MAXIMUM_POSITION,
                MINIMUM_POSITION,
                MINIMUM_POSITION,
                MOMENT_OF_INERTIA,
                HOOD_LENGTH_METERS
        );
        return TalonFXArmBuilder.buildDynamicMotionMagicArm(
                RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/DynamicMotionMagicArm",
                IDs.TalonFXIDs.NOT_HOOD2,
                IS_INVERTED,
                IS_CONTINUOUS_WRAP,
                new TalonFXFollowerConfig(),
                SYSID_ROUTINE_CONFIG,
                FEEDBACK_CONFIGS,
                REAL_SLOT,
                SIMULATION_SLOT,
                CURRENT_LIMIT,
                RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
                ARBITRARY_FEEDFORWARD,
                FORWARD_SOFTWARE_LIMIT,
                BACKWARD_SOFTWARE_LIMIT,
                hoodSimulationConstants,
                Rotation2d.fromDegrees(5),
                Rotation2d.fromDegrees(5)
        );
    }

}

