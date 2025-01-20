package frc.robot.subsystems.endEffector.factory;

import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.Robot;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.digitalinput.chooser.ChooserDigitalInput;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.rev.motors.*;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorConstants;

public class EndEffectorSparkMaxBuilder {

	private static final int NUMBER_OF_MOTORS = 1;
	public static final boolean IS_INVERTED = false;

	private static final double POSITION_CONVERSION_FACTOR = 1;
	private final static double MOMENT_OF_INERTIA = 0.001;

	private static final Double DEBOUNCE_TIME = 0.02;

	private static enum LimitSwitchDirection {
		FORWARD,
		REVERSE;
	}

	private static void configMotor(SparkMaxMotor sparkMaxMotor) {
		SparkMaxConfig config = new SparkMaxConfig();
		config.inverted(IS_INVERTED);

		sparkMaxMotor.applyConfiguration(new SparkMaxConfiguration().withSparkMaxConfig(config));
	}

	private static BrushlessSparkMAXMotor generateMotor(String logPath, SparkMaxWrapper sparkMaxWrapper) {
		SimpleMotorSimulation simulation = new SimpleMotorSimulation(
			new DCMotorSim(
				LinearSystemId.createDCMotorSystem(DCMotor.getNEO(NUMBER_OF_MOTORS), MOMENT_OF_INERTIA, 1 / POSITION_CONVERSION_FACTOR),
				DCMotor.getNEO(NUMBER_OF_MOTORS)
			)
		);

		BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, simulation, new SysIdRoutine.Config());
		configMotor(motor);
		return motor;
	}

	private static IDigitalInput generateBeamBreaker(SparkMaxWrapper sparkMaxWrapper, String logPath, LimitSwitchDirection limitSwitch) {
		if (Robot.ROBOT_TYPE.isReal()) {
			return switch (limitSwitch) {
				case FORWARD ->
					new SuppliedDigitalInput(() -> sparkMaxWrapper.getForwardLimitSwitch().isPressed(), new Debouncer(DEBOUNCE_TIME));
				case REVERSE ->
					new SuppliedDigitalInput(() -> sparkMaxWrapper.getReverseLimitSwitch().isPressed(), new Debouncer(DEBOUNCE_TIME));
			};
		} else {
			return new ChooserDigitalInput(logPath);
		}
	}

	public static EndEffector generate(String logPath) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(IDs.SparkMAXIDs.END_EFFECTOR_ROLLER_ID);

		SuppliedDoubleSignal powerSignal = new SuppliedDoubleSignal(EndEffectorConstants.LOG_PATH + "Power/", sparkMaxWrapper::get);
		SuppliedDoubleSignal currentSignal = new SuppliedDoubleSignal(
			EndEffectorConstants.LOG_PATH + "Current/",
			sparkMaxWrapper::getOutputCurrent
		);

		BrushlessSparkMAXMotor motor = generateMotor(EndEffectorConstants.LOG_PATH + "Roller/", sparkMaxWrapper);

		IDigitalInput frontDigitalInput = generateBeamBreaker(
			sparkMaxWrapper,
			EndEffectorConstants.LOG_PATH + "FrontBeamBreaker",
			LimitSwitchDirection.FORWARD
		);
		IDigitalInput backDigitalInput = generateBeamBreaker(
			sparkMaxWrapper,
			EndEffectorConstants.LOG_PATH + "BackBeamBreaker",
			LimitSwitchDirection.REVERSE
		);

		return new EndEffector(logPath, motor, powerSignal, currentSignal, frontDigitalInput, backDigitalInput);
	}

}
