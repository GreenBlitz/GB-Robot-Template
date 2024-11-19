package frc.robot.subsystems.swerve.factories.modules;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.swerve.module.maple.MapleModuleConstants;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;


public class MapleModuleGenerator {

	private static final DCMotor DRIVE_MOTOR = DCMotor.getFalcon500(1);
	private static final DCMotor STEER_MOTOR = DCMotor.getFalcon500(1);
	private static final double DRIVE_CURRENT_LIMIT_AMPS = 60;
	private static final SwerveModuleSimulation.DRIVE_WHEEL_TYPE DRIVE_WHEEL_TYPE = SwerveModuleSimulation.DRIVE_WHEEL_TYPE.RUBBER;
	private static final int GEAR_RATIO_LEVEL = 3; // L3 gear ratio

	public static SwerveModuleSimulation generate() {
		return SwerveModuleSimulation.getMark4i(DRIVE_MOTOR, STEER_MOTOR, DRIVE_CURRENT_LIMIT_AMPS, DRIVE_WHEEL_TYPE, GEAR_RATIO_LEVEL).get();
	}

	private static final PIDConstants STEER_PID_RADIANS = new PIDConstants(7);
	private static final PIDConstants DRIVE_PID_RADIANS_PER_SECOND = new PIDConstants(0.05);
	private static final SimpleMotorFeedforward DRIVE_FEED_FORWARD_RADIANS_PER_SECOND = new SimpleMotorFeedforward(0.1, 0.13);

	public static MapleModuleConstants generateMapleModuleConstants() {
		return new MapleModuleConstants(STEER_PID_RADIANS, DRIVE_PID_RADIANS_PER_SECOND, DRIVE_FEED_FORWARD_RADIANS_PER_SECOND);
	}

}
