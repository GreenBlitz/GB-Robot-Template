package frc.robot.subsystems.endEffector.factory;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.digitalinput.channeled.ChanneledDigitalInput;
import frc.robot.hardware.rev.motors.BrushlessSparkMAXMotor;
import frc.robot.hardware.rev.motors.SparkMaxDeviceID;
import frc.robot.hardware.rev.motors.SparkMaxWrapper;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorConstants;
import com.revrobotics.spark.SparkBase.ResetMode;

public class RealEndEffectorConstants {

	private static final SparkMaxDeviceID MOTOR_ID = new SparkMaxDeviceID(-1);

	private static final double VELOCITY_CONVERSION_FACTOR = 1;
	private static final double POSITION_CONVERSION_FACTOR = 1;

	private static final Double DEBOUNCE_TIME = 0.02;
	private static final int FRONT_DIGITAL_INPUT_CHANNEL = -1;
	private static final int BACK_DIGITAL_INPUT_CHANNEL = -1;

	private static void motorConfig(SparkMaxWrapper sparkMaxWrapper) {
		SparkMaxConfig config = new SparkMaxConfig();
		config.encoder.velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
		config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR);
		config.inverted(EndEffectorConstants.IS_INVERTED);
		sparkMaxWrapper.configure(config, ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
	}

	private static BrushlessSparkMAXMotor generateMotor(String logPath, SparkMaxDeviceID id) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(id);
		motorConfig(sparkMaxWrapper);
		return new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, new SysIdRoutine.Config());
	}

	public static EndEffector generate(String logPath, String motorLogPath) {
		BrushlessSparkMAXMotor motor = generateMotor(motorLogPath, MOTOR_ID);

		ChanneledDigitalInput frontDigitalInput = new ChanneledDigitalInput(
			new DigitalInput(FRONT_DIGITAL_INPUT_CHANNEL),
			new Debouncer(DEBOUNCE_TIME)
		);
		ChanneledDigitalInput backDigitalInput = new ChanneledDigitalInput(
			new DigitalInput(BACK_DIGITAL_INPUT_CHANNEL),
			new Debouncer(DEBOUNCE_TIME)
		);

		return new EndEffector(motor, frontDigitalInput, backDigitalInput, logPath);
	}

}
