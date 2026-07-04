package frc.robot.subsystems.constants.turret;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.statemachine.shooterstatehandler.TurretCalculations;
import frc.robot.subsystems.arm.ArmSimulationConstants;
import frc.robot.subsystems.arm.TalonFXArmBuilder;
import frc.robot.subsystems.arm.VelocityPositionArm;

public class TurretConstants {

	public static final String LOG_PATH = RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Turret";

	public static final boolean IS_INVERTED = false;
	public static final boolean IS_CONTINUOUS_WRAP = false;

	public static final double CURRENT_LIMIT = 40;

	public static final TalonFXFollowerConfig TALON_FX_FOLLOWER_CONFIG = new TalonFXFollowerConfig();
	public static final SysIdRoutine.Config SYS_ID_ROUTINE_CONFIG = new SysIdRoutine.Config();
	public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs();

	public static final Slot0Configs REAL_SLOTS_CONFIG = new Slot0Configs();
	public static final Slot0Configs SIMULATION_SLOTS_CONFIG = new Slot0Configs();

	static {
		FEEDBACK_CONFIGS.SensorToMechanismRatio = 70;

		REAL_SLOTS_CONFIG.kP = 600;
		REAL_SLOTS_CONFIG.kI = 0;
		REAL_SLOTS_CONFIG.kD = 0;
		REAL_SLOTS_CONFIG.kG = 0;
		REAL_SLOTS_CONFIG.kS = 0.24;
		REAL_SLOTS_CONFIG.kV = 7.02;
		REAL_SLOTS_CONFIG.kA = 0;

		SIMULATION_SLOTS_CONFIG.kP = 80;
		SIMULATION_SLOTS_CONFIG.kI = 0;
		SIMULATION_SLOTS_CONFIG.kD = 0;
		SIMULATION_SLOTS_CONFIG.kG = 0;
		SIMULATION_SLOTS_CONFIG.kS = 0;
		SIMULATION_SLOTS_CONFIG.kV = 0;
		SIMULATION_SLOTS_CONFIG.kA = 0;
	}

	public static final Rotation2d MAX_POSITION = Rotation2d.fromDegrees(120.4);
	public static final Rotation2d MIN_POSITION = Rotation2d.fromDegrees(-239.6);
	public static final Rotation2d FORWARD_SOFTWARE_LIMIT = Rotation2d.fromDegrees(90);
	public static final Rotation2d BACKWARDS_SOFTWARE_LIMIT = Rotation2d.fromDegrees(-116);

	public static final Translation3d TURRET_POSITION_RELATIVE_TO_ROBOT = new Translation3d(0.1385, -0.220, 0.45);
	public static final Rotation2d MAX_DISTANCE_FROM_LIMIT_NOT_TO_ROTATE = Rotation2d.fromDegrees(0);

	public static final Rotation2d RANGE_MIDDLE = Rotation2d.fromDegrees((MAX_POSITION.getDegrees() + MIN_POSITION.getDegrees()) / 2);
	public static final Rotation2d SCREW_MAX_RANGE_EDGE = TurretCalculations
		.getRangeEdge(MAX_POSITION, MAX_DISTANCE_FROM_LIMIT_NOT_TO_ROTATE.times(-1));
	public static final Rotation2d SCREW_MIN_RANGE_EDGE = TurretCalculations.getRangeEdge(MIN_POSITION, MAX_DISTANCE_FROM_LIMIT_NOT_TO_ROTATE);

	public static final double RESET_TURRET_VOLTAGE = 0.4;
	public static final double CURRENT_THRESHOLD_TO_RESET_POSITION = 7;
	public static final double RESET_START_IGNORANCE_TIME_SECONDS = 0.4;

	public static final double MOMENT_OF_INERTIA = 0.001;
	public static final double TURRET_RADIUS = 0.0;
	public static final double ARBITRARY_FEED_FORWARD = 0.0;

	public static VelocityPositionArm createTurret() {
		ArmSimulationConstants turretSimulationConstants = new ArmSimulationConstants(
			MAX_POSITION,
			MIN_POSITION,
			MIN_POSITION,
			MOMENT_OF_INERTIA,
			TURRET_RADIUS
		);
		return TalonFXArmBuilder.buildVelocityPositionArm(
			LOG_PATH,
			IDs.TalonFXIDs.TURRET,
			IS_INVERTED,
			IS_CONTINUOUS_WRAP,
			TALON_FX_FOLLOWER_CONFIG,
			SYS_ID_ROUTINE_CONFIG,
			FEEDBACK_CONFIGS,
			REAL_SLOTS_CONFIG,
			SIMULATION_SLOTS_CONFIG,
			CURRENT_LIMIT,
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			ARBITRARY_FEED_FORWARD,
			FORWARD_SOFTWARE_LIMIT,
			BACKWARDS_SOFTWARE_LIMIT,
			turretSimulationConstants
		);
	}

}
