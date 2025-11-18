package frc.robot.subsystems.constants.hood;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.geometry.Rotation2d;

public class HoodConstants {
	public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs();
	public static final Slot0Configs REAL_SLOT = new Slot0Configs();
	public static final Slot0Configs SIMULATION_SLOT = new Slot0Configs();

	static {
	REAL_SLOT.kP = HoodConstants.kP;
	REAL_SLOT.kI = HoodConstants.kI;
	REAL_SLOT.kD = HoodConstants.kD;
	REAL_SLOT.kV = HoodConstants.kV;
	REAL_SLOT.kG = HoodConstants.kG;
	REAL_SLOT.kA = HoodConstants.kA;
	REAL_SLOT.kS = HoodConstants.kS;
	REAL_SLOT.GravityType = GravityTypeValue.Arm_Cosine;

	SIMULATION_SLOT.kP = HoodConstants.SIM_kP;
	SIMULATION_SLOT.kI = HoodConstants.SIM_kI;
	SIMULATION_SLOT.kD = HoodConstants.SIM_kD;
	SIMULATION_SLOT.kG = HoodConstants.SIM_kG;
	SIMULATION_SLOT.kS = HoodConstants.SIM_kS;
	SIMULATION_SLOT.GravityType = GravityTypeValue.Arm_Cosine;

	FEEDBACK_CONFIGS.RotorToSensorRatio = HoodConstants.GEAR_RATIO;
	FEEDBACK_CONFIGS.SensorToMechanismRatio = 1;
	FEEDBACK_CONFIGS.FeedbackRemoteSensorID = 20;
	FEEDBACK_CONFIGS.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
	}

	public static final boolean IS_INVERTED = false;
	public static final double GEAR_RATIO = 450.0 / 7.0;
	public static final double kP = 28;
	public static final double kI = 0;
	public static final double kD = 0;
	public static final double kV = 0.065;
	public static final double kS = 0.37;
	public static final double kA = 0.5209;
	public static final double kG = 9.0000095367432;
	public static final double SIM_kP = 70;
	public static final double SIM_kI = 0;
	public static final double SIM_kD = 0;
	public static final double SIM_kS = 0;
	public static final double SIM_kG = 0;
	public static final double ARBITRARY_FEEDFORWARD = 0;
	public static final double CURRENT_LIMIT = 40;
	public static final double MOMENT_OF_INERTIA = 0.001;
	public static final double ARM_LENGTH_METERS = 0.3;
	public static final Rotation2d FORWARD_SOFTWARE_LIMIT = Rotation2d.fromDegrees(246-16);
	public static final Rotation2d BACKWARD_SOFTWARE_LIMIT = Rotation2d.fromDegrees(-24);
	public static final Rotation2d DEFAULT_MAX_ACCELERATION_PER_SECOND_SQUARE = Rotation2d.fromRotations(3);
	public static final Rotation2d DEFAULT_MAX_VELOCITY_PER_SECOND = Rotation2d.fromRotations(3);
	public static final Rotation2d STARTING_POSITION = Rotation2d.fromRotations(-27);

}

