package frc.robot.subsystems.swerve.factories.modules;

import frc.robot.Robot;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.factories.modules.constants.ModuleConstantsFactory;
import frc.robot.subsystems.swerve.factories.modules.drive.DriveFactory;
import frc.robot.subsystems.swerve.factories.modules.encoder.EncoderFactory;
import frc.robot.subsystems.swerve.factories.modules.steer.SteerFactory;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.maple.MapleModule;
import frc.robot.subsystems.swerve.module.hardware.HardwareModule;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.module.Modules;


public class ModulesFactory {

	private static MapleModule createMapleModule(SwerveType swerveType, ModuleUtils.ModulePosition modulePosition) {
		return new MapleModule(
			ModuleConstantsFactory.create(swerveType, modulePosition),
			MapleModuleGenerator.generateMapleModuleConstants(),
			MapleModuleGenerator.generate()
		);
	}

	private static HardwareModule createHardwareModule(SwerveType swerveType, ModuleUtils.ModulePosition modulePosition) {
		IAngleEncoder angleEncoder = EncoderFactory.createEncoder(swerveType, modulePosition);
		ControllableMotor steer = SteerFactory.createSteer(swerveType, modulePosition);
		ControllableMotor drive = DriveFactory.createDrive(swerveType, modulePosition);

		return new HardwareModule(
			ModuleConstantsFactory.create(swerveType, modulePosition),
			angleEncoder,
			EncoderFactory.createSignals(swerveType, angleEncoder),
			steer,
			SteerFactory.createRequests(swerveType),
			SteerFactory.createSignals(swerveType, steer),
			drive,
			DriveFactory.createRequests(swerveType),
			DriveFactory.createSignals(swerveType, drive)
		);
	}

	private static Module createModule(SwerveType swerveType, ModuleUtils.ModulePosition modulePosition, boolean isMaple) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> createHardwareModule(swerveType, modulePosition);
			case SIMULATION -> isMaple ? createMapleModule(swerveType, modulePosition) : createHardwareModule(swerveType, modulePosition);
		};
	}

	public static Modules create(SwerveType swerveType, boolean isMaple) {
		return new Modules(
			swerveType.getLogPath(),
			createModule(swerveType, ModuleUtils.ModulePosition.FRONT_LEFT, isMaple),
			createModule(swerveType, ModuleUtils.ModulePosition.FRONT_RIGHT, isMaple),
			createModule(swerveType, ModuleUtils.ModulePosition.BACK_LEFT, isMaple),
			createModule(swerveType, ModuleUtils.ModulePosition.BACK_RIGHT, isMaple)
		);
	}

}
