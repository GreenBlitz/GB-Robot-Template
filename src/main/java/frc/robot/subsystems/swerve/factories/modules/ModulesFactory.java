package frc.robot.subsystems.swerve.factories.modules;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.factories.modules.constants.ModuleConstantsFactory;
import frc.robot.subsystems.swerve.factories.modules.drive.DriveFactory;
import frc.robot.subsystems.swerve.factories.modules.encoder.EncoderFactory;
import frc.robot.subsystems.swerve.factories.modules.steer.SteerFactory;
import frc.robot.subsystems.swerve.module.IModule;
import frc.robot.subsystems.swerve.module.MapleModule;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.module.Modules;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;

import java.util.function.Supplier;


public class ModulesFactory {

	private static IModule
		createModule(SwerveType swerveType, ModuleUtils.ModulePosition modulePosition, SwerveModuleSimulation moduleSimulation) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL ->
				new Module(
					ModuleConstantsFactory.create(swerveType, modulePosition),
					EncoderFactory.create(swerveType, modulePosition),
					SteerFactory.create(swerveType, modulePosition),
					DriveFactory.create(swerveType, modulePosition)
				);
			case SIMULATION -> new MapleModule(ModuleConstantsFactory.create(swerveType, modulePosition), moduleSimulation);
		};
	}

	public static Modules create(SwerveType swerveType, SwerveDriveSimulation swerveDriveSimulation) {
		return new Modules(
			swerveType.getLogPath(),
			createModule(swerveType, ModuleUtils.ModulePosition.FRONT_LEFT, swerveDriveSimulation.getModules()[0]),
			createModule(swerveType, ModuleUtils.ModulePosition.FRONT_RIGHT, swerveDriveSimulation.getModules()[1]),
			createModule(swerveType, ModuleUtils.ModulePosition.BACK_LEFT, swerveDriveSimulation.getModules()[2]),
			createModule(swerveType, ModuleUtils.ModulePosition.BACK_RIGHT, swerveDriveSimulation.getModules()[3])
		);
	}

}
