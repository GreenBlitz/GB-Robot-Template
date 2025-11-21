package frc.robot.subsystems.constants.fourBar;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;

public class FourBarConstants {

	public static final String LOG_PATH = "FourBar";
	public static final int CURRENT_LIMIT = 40;
	public static final double MOMENT_OF_INERTIA = 0.001;
	public static final double ARM_LENGTH_METERS = 0.3;
	public static final double ARBITRARY_FEED_FORWARD = 0.0;
	public static final boolean IS_INVERTED = false;
	public static final TalonFXFollowerConfig TALON_FX_FOLLOWER_CONFIG = new TalonFXFollowerConfig();
	public static final SysIdRoutine.Config SYS_ID_ROUTINE = new SysIdRoutine.Config();
	public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs();
	public static final Slot0Configs REAL_SLOT = new Slot0Configs();
	public static final Slot0Configs SIMULATION_SLOT = new Slot0Configs();

	static {
		REAL_SLOT.kP = 28;
		REAL_SLOT.kI = 0;
		REAL_SLOT.kD = 0;
		REAL_SLOT.kS = 0.065;
		REAL_SLOT.kG = 0.37;
		REAL_SLOT.kV = 9.0000095367432;
		REAL_SLOT.kA = 0.5209;
		REAL_SLOT.GravityType = GravityTypeValue.Arm_Cosine;

		SIMULATION_SLOT.kP = 70;
		SIMULATION_SLOT.kI = 0;
		SIMULATION_SLOT.kD = 0;
		SIMULATION_SLOT.kG = 0;
		SIMULATION_SLOT.kS = 0;
		SIMULATION_SLOT.GravityType = GravityTypeValue.Arm_Cosine;

		FEEDBACK_CONFIGS.RotorToSensorRatio = 1;
		FEEDBACK_CONFIGS.SensorToMechanismRatio = 450 / 7.0;
	}

	public static final Rotation2d FORWARD_SOFTWARE_LIMITS = Rotation2d.fromDegrees(193);
	public static final Rotation2d BACKWARD_SOFTWARE_LIMITS = Rotation2d.fromDegrees(-24);
	public static final Rotation2d MAXIMUM_POSITION = Rotation2d.fromDegrees(220);
	public static final Rotation2d MINIMUM_POSITION = Rotation2d.fromDegrees(-27);
	public static final Rotation2d MAX_ACCELERATION_ROTATION2D_PER_SECONDS_SQUARE = Rotation2d.fromRotations(3);
	public static final Rotation2d MAX_VELOCITY_ROTATION2D_PER_SECONDS = Rotation2d.fromRotations(3);

}
