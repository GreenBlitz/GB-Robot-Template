package frc.robot.subsystems.endEffector.factory;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.hardware.digitalinput.channeled.ChanneledDigitalInput;
import frc.robot.hardware.digitalinput.chooser.ChooserDigitalInput;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.rev.motors.*;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorConstants;

public class RealEndEffectorConstants {

	private static final int NUMBER_OF_MOTORS = 1;

	private static final double VELOCITY_CONVERSION_FACTOR = 1;
	private static final double POSITION_CONVERSION_FACTOR = 1;
	private final static double MOMENT_OF_INERTIA = 0.001;

	private static final Double DEBOUNCE_TIME = 0.02;
	private static final int FRONT_DIGITAL_INPUT_CHANNEL = -1;
	private static final int BACK_DIGITAL_INPUT_CHANNEL = -1;

	private static void motorConfig(SparkMaxMotor sparkMaxMotor) {
		SparkMaxConfig config = new SparkMaxConfig();
		config.encoder.velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);
		config.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR);
		config.inverted(EndEffectorConstants.IS_INVERTED);
		sparkMaxMotor.applyConfiguration(new SparkMaxConfiguration().withSparkMaxConfig(config));
	}

	private static BrushlessSparkMAXMotor generateRealMotor(String logPath, SparkMaxDeviceID id) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(id);
		BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, new SysIdRoutine.Config());
		motorConfig(motor);
		return motor;
	}

	private static BrushlessSparkMAXMotor generateSimMotor(String logPath) {
		SimpleMotorSimulation simulation = new SimpleMotorSimulation(
			new DCMotorSim(
				LinearSystemId.createDCMotorSystem(DCMotor.getNEO(NUMBER_OF_MOTORS), MOMENT_OF_INERTIA, POSITION_CONVERSION_FACTOR),
				DCMotor.getNEO(NUMBER_OF_MOTORS)
			)
		);

		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(IDs.SparkMAXIDs.END_EFFECTOR_ROLLER_ID);
		BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, simulation, new SysIdRoutine.Config());
		motorConfig(motor);
		return motor;
	}

	public static EndEffector generateReal(String logPath, String motorLogPath) {
		BrushlessSparkMAXMotor motor = generateRealMotor(motorLogPath, IDs.SparkMAXIDs.END_EFFECTOR_ROLLER_ID);

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

	public static EndEffector generateSim(String logPath, String motorLogPath) {
		BrushlessSparkMAXMotor motor = generateSimMotor(motorLogPath);

		ChooserDigitalInput frontDigitalInput = new ChooserDigitalInput(EndEffectorConstants.LOG_PATH + "FrontBeamBreaker");
		ChooserDigitalInput backDigitalInput = new ChooserDigitalInput(EndEffectorConstants.LOG_PATH + "BackBeamBreaker");


		return new EndEffector(motor, frontDigitalInput, backDigitalInput, logPath);
	}

}
