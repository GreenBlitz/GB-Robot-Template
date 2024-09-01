package frc.robot.subsystems.swerve.factories.modules.drive;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.swerve.factories.modules.constants.SimulationModuleConstants;
import frc.robot.subsystems.swerve.factories.swerveconstants.SimulationSwerveConstants;
import frc.robot.subsystems.swerve.modules.drive.simulation.SimulationDriveConstants;
import frc.utils.Conversions;

class DriveSimulationConstants {

	private static final double DRIVE_GEAR_RATIO = 6.12;

	private static final double DRIVE_MOMENT_OF_INERTIA = 0.001;

	private static final boolean ENABLE_FOC_DRIVE = true;

	protected static SimulationDriveConstants getDriveConstants() {
		return new SimulationDriveConstants(
			new DCMotorSim(DCMotor.getFalcon500Foc(1), DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA),
			ENABLE_FOC_DRIVE,
			Conversions.distanceToAngle(
				SimulationSwerveConstants.VELOCITY_AT_12_VOLTS_METERS_PER_SECOND,
				SimulationModuleConstants.WHEEL_DIAMETER_METERS
			),
			new SysIdRoutine.Config()
		);
	}

}
