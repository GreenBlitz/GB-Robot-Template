package frc.robot.subsystems.endEffector.factory;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.RobotType;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.digitalinput.channeled.ChanneledDigitalInput;
import frc.robot.hardware.digitalinput.chooser.ChooserDigitalInput;
import frc.robot.hardware.interfaces.IMotor;
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

	private static BrushlessSparkMAXMotor generateMotor(String logPath, SparkMaxDeviceID id) {
		SimpleMotorSimulation simulation = new SimpleMotorSimulation(
			new DCMotorSim(
				LinearSystemId.createDCMotorSystem(DCMotor.getNEO(NUMBER_OF_MOTORS), MOMENT_OF_INERTIA, POSITION_CONVERSION_FACTOR),
				DCMotor.getNEO(NUMBER_OF_MOTORS)
			)
		);

		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(id);
		BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, simulation, new SysIdRoutine.Config());
		motorConfig(motor);
		return motor;
	}

	private static IDigitalInput generateBeamBreaker(int channel, String logPath) {
		if (RobotType.REAL.isReal()) {
			return new ChanneledDigitalInput(new DigitalInput(channel), new Debouncer(DEBOUNCE_TIME));
		} else {
			return new ChooserDigitalInput(logPath);
		}
	}

	public static EndEffector generate(String logPath, String motorLogPath) {
		IMotor motor = generateMotor(motorLogPath, IDs.SparkMAXIDs.END_EFFECTOR_ROLLER_ID);

		IDigitalInput frontDigitalInput = generateBeamBreaker(FRONT_DIGITAL_INPUT_CHANNEL, EndEffectorConstants.LOG_PATH + "FrontBeamBreaker");
		IDigitalInput backDigitalInput = generateBeamBreaker(BACK_DIGITAL_INPUT_CHANNEL, EndEffectorConstants.LOG_PATH + "BackBeamBreaker");

		return new EndEffector(motor, frontDigitalInput, backDigitalInput, logPath);
	}

}
