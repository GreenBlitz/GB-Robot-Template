package frc.robot.subsystems.endEffector.factory;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.digitalinput.channeled.ChanneledDigitalInput;
import frc.robot.hardware.rev.motors.BrushlessSparkMAXMotor;
import frc.robot.hardware.rev.motors.SparkMaxDeviceID;
import frc.robot.hardware.rev.motors.SparkMaxWrapper;
import frc.robot.hardware.rev.request.SparkMaxRequest;
import frc.robot.hardware.rev.request.SparkMaxRequestBuilder;
import frc.robot.subsystems.endEffector.EndEffector;

public class RealEndEffectorConstants {

	private static final SparkMaxDeviceID MOTOR_ID = new SparkMaxDeviceID(-1);
	private static final int FRONT_DIGITAL_INPUT_CHANNEL = -1;
	private static final int BACK_DIGITAL_INPUT_CHANNEL = -1;
	private static final Double DEBOUNCE_TIME = 0.02;

	private static BrushlessSparkMAXMotor generateMotor(String logPath, SparkMaxDeviceID id) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(id);
		return new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, new SysIdRoutine.Config());
	}

	public static EndEffector generate(String logPath, String motorLogPath) {
		BrushlessSparkMAXMotor motor = generateMotor(motorLogPath, MOTOR_ID);
		SparkMaxRequest<Double> powerRequest = SparkMaxRequestBuilder.build(0.0, SparkBase.ControlType.kDutyCycle, 0);

		ChanneledDigitalInput frontDigitalInput = new ChanneledDigitalInput(
			new DigitalInput(FRONT_DIGITAL_INPUT_CHANNEL),
			new Debouncer(DEBOUNCE_TIME)
		);
		ChanneledDigitalInput backDigitalInput = new ChanneledDigitalInput(
			new DigitalInput(BACK_DIGITAL_INPUT_CHANNEL),
			new Debouncer(DEBOUNCE_TIME)
		);

		return new EndEffector(motor, frontDigitalInput, backDigitalInput, powerRequest, logPath);
	}

}
