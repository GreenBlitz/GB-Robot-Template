package frc.robot.subsystems.swerve.factories.modules;

import edu.wpi.first.math.system.plant.DCMotor;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

import java.util.function.Supplier;

public class SimulationModuleGenerator {

	private static final DCMotor DRIVE_MOTOR = DCMotor.getFalcon500(1);
	private static final DCMotor STEER_MOTOR = DCMotor.getFalcon500(1);
	private static final double DRIVE_CURRENT_LIMIT_AMPS = 60;
	private static final SwerveModuleSimulation.DRIVE_WHEEL_TYPE DRIVE_WHEEL_TYPE = SwerveModuleSimulation.DRIVE_WHEEL_TYPE.RUBBER;
	private static final int GEAR_RATIO_LEVEL = 3; // L3

	public static Supplier<SwerveModuleSimulation> generate() {
		return SwerveModuleSimulation.getMark4(DRIVE_MOTOR, STEER_MOTOR, DRIVE_CURRENT_LIMIT_AMPS, DRIVE_WHEEL_TYPE, GEAR_RATIO_LEVEL);
	}

}
