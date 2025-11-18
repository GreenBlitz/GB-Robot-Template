package frc.robot.subsystems.constants.turret;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;

import java.io.ObjectInputFilter;

public class TurretConstants {

    public static final String LOG_PATH = "turret";
    public static final Phoenix6DeviceID DEVICE_ID = IDs.TalonFXIDs.turretID;
    public static final boolean IS_INVERTED = false;
    public static final TalonFXFollowerConfig TALON_FX_FOLLOWER_CONFIG = new TalonFXFollowerConfig();
    public static final SysIdRoutine.Config SYS_ID_ROUTINE_CONFIG = new SysIdRoutine.Config();
    public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs();
    public static final Slot0Configs REAL_SLOTS_CONFIG = new Slot0Configs();
    public static final Slot0Configs SIMULATION_SLOTS_CONFIG = new Slot0Configs();
    public static final double CURRENT_LIMIT = 40;
    public static final int SIGNALS_FREQUENCY = (int) RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ;
    public static final double MOMENT_OF_INERTIA = 0.001;
    public static final double ARM_LENGTH = 0.04;

}
