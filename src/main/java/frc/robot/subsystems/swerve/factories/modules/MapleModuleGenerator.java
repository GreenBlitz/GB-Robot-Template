package frc.robot.subsystems.swerve.factories.modules;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.swerve.module.maple.MapleModuleConstants;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;


public class MapleModuleGenerator {

	private static final DCMotor DRIVE_MOTOR = DCMotor.getFalcon500(1);
	private static final DCMotor STEER_MOTOR = DCMotor.getFalcon500(1);
	private static final double WHEEL_COF = 1.2;
	private static final Voltage kSteerFrictionVoltage = Volts.of(0.25);
	private static final Voltage kDriveFrictionVoltage = Volts.of(0.25);
	private static final double kSteerInertia = 0.01;
	private static final Distance kWheelRadius = Inches.of(2.167);


	public static SwerveModuleSimulation generate() {
		return new SwerveModuleSimulation(
			DRIVE_MOTOR,
			STEER_MOTOR,
			6.12,
			150.0 / 7.0,
			kDriveFrictionVoltage,
			kSteerFrictionVoltage,
			kWheelRadius,
			KilogramSquareMeters.of(kSteerInertia),
			WHEEL_COF
		);
	}

	private static final PIDConstants STEER_PID_RADIANS = new PIDConstants(7);
	private static final PIDConstants DRIVE_PID_RADIANS_PER_SECOND = new PIDConstants(0.05);
	private static final SimpleMotorFeedforward DRIVE_FEED_FORWARD_RADIANS_PER_SECOND = new SimpleMotorFeedforward(0.1, 0.13);

	public static MapleModuleConstants generateMapleModuleConstants() {
		return new MapleModuleConstants(STEER_PID_RADIANS, DRIVE_PID_RADIANS_PER_SECOND, DRIVE_FEED_FORWARD_RADIANS_PER_SECOND);
	}

}
