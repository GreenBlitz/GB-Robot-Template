package frc.robot.autonomous;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.factories.constants.RealSwerveConstants;
import frc.robot.subsystems.swerve.factories.modules.drive.KrakenX60DriveBuilder;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtil;


public class AutonomousConstants {

	public static final String LOG_PATH_PREFIX = "Autonomous";

	public static final double PATHFINDING_DEADBAND_METERS = 0.5;

	public static PathConstraints getRealTimeConstraints(Swerve swerve) {
		return new PathConstraints(
			swerve.getConstants().velocityAt12VoltsMetersPerSecond(),
			RealSwerveConstants.ACCELERATION_AT_12_VOLTS_METERS_PER_SECOND_SQUARED,
			swerve.getConstants().maxRotationalVelocityPerSecond().getRadians(),
			4
		);
	}

	public static RobotConfig getRobotConfig(Robot robot) {
		return new RobotConfig(
			RobotConstants.MASS_KILOGRAM,
			RobotConstants.MOMENT_OF_INERTIA_KILOGRAM_METERS_SQUARED,
			new ModuleConfig(
				robot.getSwerve().getModules().getModule(ModuleUtil.ModulePosition.FRONT_LEFT).getModuleConstants().wheelDiameterMeters() / 2,
				robot.getSwerve().getConstants().velocityAt12VoltsMetersPerSecond(),
				ModuleConstants.COEFFICIENT_OF_FRICTION,
				DCMotor.getKrakenX60Foc(ModuleConstants.NUMBER_OF_DRIVE_MOTORS),
				KrakenX60DriveBuilder.GEAR_RATIO,
				KrakenX60DriveBuilder.SLIP_CURRENT,
				ModuleConstants.NUMBER_OF_DRIVE_MOTORS
			),
			robot.getSwerve().getModules().getModulePositionsFromCenterMeters()
		);
	}

}
