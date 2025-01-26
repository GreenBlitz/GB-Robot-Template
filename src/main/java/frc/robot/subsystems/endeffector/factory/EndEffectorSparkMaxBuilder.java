package frc.robot.subsystems.endeffector.factory;

import com.revrobotics.spark.config.LimitSwitchConfig;
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
import frc.robot.subsystems.endeffector.EndEffector;

public class EndEffectorSparkMaxBuilder {

	private static final int NUMBER_OF_MOTORS = 1;
	private static final boolean IS_INVERTED = false;
	private static final boolean SET_BRAKE = false;
	private static final int CURRENT_LIMIT = 30;

	private static final double POSITION_CONVERSION_FACTOR = 1;
	private static final double MOMENT_OF_INERTIA = 0.001;

	private static final Double DEBOUNCE_TIME_SECONDS = 0.1;

	private static void configMotor(SparkMaxMotor sparkMaxMotor) {
		SparkMaxConfig config = new SparkMaxConfig();
		config.inverted(IS_INVERTED);
		config.smartCurrentLimit(CURRENT_LIMIT);

		config.limitSwitch.forwardLimitSwitchEnabled(false);
		config.limitSwitch.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);

		config.limitSwitch.reverseLimitSwitchEnabled(false);
		config.limitSwitch.reverseLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);

		sparkMaxMotor.applyConfiguration(new SparkMaxConfiguration().withSparkMaxConfig(config));
		sparkMaxMotor.setBrake(SET_BRAKE);
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

	private static IDigitalInput generateBeamBreaker(SparkMaxWrapper sparkMaxWrapper, String name, LimitSwitchPort limitSwitch) {
		if (Robot.ROBOT_TYPE.isSimulation()) {
			return new ChooserDigitalInput(name);
		}

		return switch (limitSwitch) {
			case FORWARD ->
				new SuppliedDigitalInput(() -> sparkMaxWrapper.getForwardLimitSwitch().isPressed(), new Debouncer(DEBOUNCE_TIME_SECONDS));
			case REVERSE ->
				new SuppliedDigitalInput(() -> sparkMaxWrapper.getReverseLimitSwitch().isPressed(), new Debouncer(DEBOUNCE_TIME_SECONDS));
		};
	}

	public static EndEffector generate(String logPath) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(IDs.SparkMAXIDs.END_EFFECTOR);

		SuppliedDoubleSignal powerSignal = new SuppliedDoubleSignal("power", sparkMaxWrapper::get);
		SuppliedDoubleSignal currentSignal = new SuppliedDoubleSignal("current", sparkMaxWrapper::getOutputCurrent);

		BrushlessSparkMAXMotor motor = generateMotor(logPath + "/Roller", sparkMaxWrapper);

		IDigitalInput frontDigitalInput = generateBeamBreaker(sparkMaxWrapper, "FrontBeamBreaker", LimitSwitchPort.FORWARD);
		IDigitalInput backDigitalInput = generateBeamBreaker(sparkMaxWrapper, "BackBeamBreaker", LimitSwitchPort.REVERSE);

		return new EndEffector(logPath, motor, powerSignal, currentSignal, frontDigitalInput, backDigitalInput);
	}

}
