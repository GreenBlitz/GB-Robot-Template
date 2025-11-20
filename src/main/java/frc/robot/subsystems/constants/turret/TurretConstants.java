package frc.robot.subsystems.constants.turret;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotConstants;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;

public class TurretConstants {

	public static final String LOG_PATH = RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/turret";
	public static final boolean IS_INVERTED = false;
	public static final TalonFXFollowerConfig TALON_FX_FOLLOWER_CONFIG = new TalonFXFollowerConfig();
	public static final SysIdRoutine.Config SYS_ID_ROUTINE_CONFIG = new SysIdRoutine.Config();
	public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs();

	public static final Slot0Configs REAL_SLOTS_CONFIG = new Slot0Configs();
	public static final Slot0Configs SIMULATION_SLOTS_CONFIG = new Slot0Configs();

	static {
		REAL_SLOTS_CONFIG.kP = 1;
		REAL_SLOTS_CONFIG.kI = 0;
		REAL_SLOTS_CONFIG.kD = 0;
		REAL_SLOTS_CONFIG.kG = 0;
		REAL_SLOTS_CONFIG.kS = 0;
		REAL_SLOTS_CONFIG.kV = 0;
		REAL_SLOTS_CONFIG.kA = 0;


		SIMULATION_SLOTS_CONFIG.kP = 10;
		SIMULATION_SLOTS_CONFIG.kI = 0;
		SIMULATION_SLOTS_CONFIG.kD = 0;
		SIMULATION_SLOTS_CONFIG.kG = 0;
		SIMULATION_SLOTS_CONFIG.kS = 0;
		SIMULATION_SLOTS_CONFIG.kV = 0;
		SIMULATION_SLOTS_CONFIG.kA = 0;

		FEEDBACK_CONFIGS.withSensorToMechanismRatio(79.2);
	}

	public static final double CURRENT_LIMIT = 40;
	public static final double MOMENT_OF_INERTIA = 0.001;
	public static final double ARM_LENGTH = 0.0;
	public static final double ARBITRARY_FEED_FORWARD = 0.0;
	public static final Rotation2d FORWARD_SOFTWARE_LIMIT = Rotation2d.fromDegrees(257.0);
	public static final Rotation2d BACKWARDS_SOFTWARE_LIMIT = Rotation2d.fromDegrees(4.0);
	public static final Rotation2d MIN_POSITION = Rotation2d.fromDegrees(3.0);
	public static final Rotation2d DEFAULT_MAX_ACCELERATION_PER_SECOND_SQUARE = Rotation2d.fromRotations(3.0);
	public static final Rotation2d DEFAULT_MAX_VELOCITY_PER_SECOND = Rotation2d.fromRotations(3.0);

}
