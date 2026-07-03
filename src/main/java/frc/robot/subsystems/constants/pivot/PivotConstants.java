package frc.robot.subsystems.constants.pivot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.subsystems.arm.ArmSimulationConstants;
import frc.robot.subsystems.arm.CurrentControlArm;
import frc.robot.subsystems.arm.TalonFXArmBuilder;

public class PivotConstants {

	public static final String LOG_PATH = RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Pivot";

	public static final boolean IS_INVERTED = false;
	public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs();
	public static final boolean IS_CONTINUOUS_WRAP = false;

	public static final int CURRENT_LIMIT = 25;

	public static final Slot0Configs REAL_SLOT = new Slot0Configs();
	public static final Slot0Configs SIMULATION_SLOT = new Slot0Configs();
	public static final TalonFXFollowerConfig TALON_FX_FOLLOWER_CONFIG = new TalonFXFollowerConfig();

	static {
		FEEDBACK_CONFIGS.RotorToSensorRatio = 1;
		FEEDBACK_CONFIGS.SensorToMechanismRatio = 81 + 2.0 / 3.0;

		REAL_SLOT.kP = 60;
		REAL_SLOT.kI = 0;
		REAL_SLOT.kD = 0;
		REAL_SLOT.kS = 0.05;
		REAL_SLOT.kG = 0.3 / Rotation2d.fromDegrees(44).getCos();
		REAL_SLOT.kV = 0;
		REAL_SLOT.kA = 0;
		REAL_SLOT.GravityType = GravityTypeValue.Arm_Cosine;

		SIMULATION_SLOT.kP = 100;
		SIMULATION_SLOT.kI = 0;
		SIMULATION_SLOT.kD = 0;
		SIMULATION_SLOT.kG = 0;
		SIMULATION_SLOT.kS = 0;
		SIMULATION_SLOT.GravityType = GravityTypeValue.Arm_Cosine;
	}

	public static final Rotation2d BACKLASH_OFFSET = Rotation2d.fromDegrees(19);
	public static final Rotation2d MAXIMUM_POSITION = Rotation2d.fromDegrees(89.4).plus(BACKLASH_OFFSET);
	public static final Rotation2d MINIMUM_POSITION = Rotation2d.fromDegrees(19.35);

	public static final Rotation2d FORWARD_SOFTWARE_LIMIT = Rotation2d.fromDegrees(78).plus(BACKLASH_OFFSET);
	public static final Rotation2d BACKWARD_SOFTWARE_LIMIT = Rotation2d.fromDegrees(25);

	public static final double PIVOT_RESET_VOLTAGE = 2;

	public static final double CURRENT_THRESHOLD_TO_RESET_POSITION = 20;

	public static final double SLOW_CLOSE_VOLTAGE = 2;
	public static final Rotation2d PIVOT_POSITION_FOR_SLOW_CLOSE = Rotation2d.fromDegrees(85);

	public static final double PIVOT_LENGTH_METERS = 0.3;
	public static final double MOMENT_OF_INERTIA = 0.001;
	public static final SysIdRoutine.Config SYS_ID_ROUTINE = new SysIdRoutine.Config();

	public static final Rotation2d POSITION_TOLERANCE_TO_START_REST = Rotation2d.fromDegrees(7);

	public static final double REST_CURRENT = 0;
	public static final double HARD_OPEN = -8;

	public static CurrentControlArm createPivot() {
		ArmSimulationConstants pivotSimConstant = new ArmSimulationConstants(
			MAXIMUM_POSITION,
			MINIMUM_POSITION,
			MAXIMUM_POSITION,
			MOMENT_OF_INERTIA,
			PIVOT_LENGTH_METERS
		);
		return TalonFXArmBuilder.buildCurrentControlArm(
			LOG_PATH,
			IDs.TalonFXIDs.PIVOT,
			IS_INVERTED,
			IS_CONTINUOUS_WRAP,
			TALON_FX_FOLLOWER_CONFIG,
			SYS_ID_ROUTINE,
			FEEDBACK_CONFIGS,
			REAL_SLOT,
			SIMULATION_SLOT,
			CURRENT_LIMIT,
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			FORWARD_SOFTWARE_LIMIT,
			BACKWARD_SOFTWARE_LIMIT,
			pivotSimConstant
		);
	}

}
