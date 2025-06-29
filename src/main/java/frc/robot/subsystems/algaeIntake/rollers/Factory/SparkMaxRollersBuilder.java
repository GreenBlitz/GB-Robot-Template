package frc.robot.subsystems.algaeIntake.rollers.Factory;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.rev.motors.BrushlessSparkMAXMotor;
import frc.robot.hardware.rev.motors.SparkMaxConfiguration;
import frc.robot.hardware.rev.motors.SparkMaxWrapper;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.algaeIntake.rollers.Rollers;

public class SparkMaxRollersBuilder {

	private static final int NUM_MOTORS = 1;
	private static final double MOMENT_OF_INERTIA = 0.02;
	private static final double GEAR_RATIO = 10;
	private static final boolean IS_INVERTED = false;
	private static final int ALGAE_SENSOR_CHANNEL = 4;


	private static BrushlessSparkMAXMotor generateMotor(String logPath, SparkMaxWrapper wrapper) {
		SimpleMotorSimulation sim = new SimpleMotorSimulation(
			new DCMotorSim(
				LinearSystemId.createDCMotorSystem(DCMotor.getNEO(NUM_MOTORS), MOMENT_OF_INERTIA, GEAR_RATIO),
				DCMotor.getNEO(NUM_MOTORS)
			)
		);

		return new BrushlessSparkMAXMotor(logPath + "/Motor", wrapper, sim, new SysIdRoutine.Config());
	}

	private static SparkMaxConfiguration generateMotorConfig() {
		SparkMaxConfiguration config = new SparkMaxConfiguration();

		config.getSparkMaxConfig().inverted(IS_INVERTED);

		config.getSparkMaxConfig().encoder.velocityConversionFactor(GEAR_RATIO);
		config.getSparkMaxConfig().encoder.positionConversionFactor(GEAR_RATIO);

		config.getSparkMaxConfig().smartCurrentLimit(40);

		return config;
	}

	public static Rollers generate(String logPath) {
		SparkMaxWrapper wrapper = new SparkMaxWrapper(IDs.SparkMAXIDs.ROLLERS);
		BrushlessSparkMAXMotor rollers = generateMotor(logPath, wrapper);
		rollers.applyConfiguration(generateMotorConfig());

		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("Voltage", wrapper::getVoltage);
		SuppliedDoubleSignal powerSignal = new SuppliedDoubleSignal("Power", wrapper::get);

		return new Rollers(logPath, rollers, voltageSignal, powerSignal);
	}

}
