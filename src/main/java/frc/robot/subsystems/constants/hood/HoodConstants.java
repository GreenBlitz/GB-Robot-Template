package frc.robot.subsystems.constants.hood;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotConstants;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class HoodConstants {
	
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
	public static final double CURRENT_LIMIT = 2000;
	public static final double MOMENT_OF_INERTIA = 0.001;
	public static final double ARM_LENGTH_METERS = 0.3;
	public static final Rotation2d FORWARD_SOFTWARE_LIMIT = Rotation2d.fromDegrees(246-16);
	public static final Rotation2d BACKWARD_SOFTWARE_LIMIT = Rotation2d.fromDegrees(-24 -16);
	public static final Rotation2d DEFAULT_MAX_ACCELERATION_PER_SECOND_SQUARE = Rotation2d.fromDegrees(3);
	public static final Rotation2d DEFAULT_MAX_VELOCITY_PER_SECOND = Rotation2d.fromDegrees(3);

}

