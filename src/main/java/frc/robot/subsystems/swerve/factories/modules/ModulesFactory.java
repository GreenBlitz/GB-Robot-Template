package frc.robot.subsystems.swerve.factories.modules;

import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.factories.modules.constants.ModuleConstantsFactory;
import frc.robot.subsystems.swerve.factories.modules.drive.DriveFactory;
import frc.robot.subsystems.swerve.factories.modules.encoder.EncoderFactory;
import frc.robot.subsystems.swerve.factories.modules.steer.SteerFactory;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.module.Modules;


public class ModulesFactory {

	private static Module createModule(SwerveType swerveType, ModuleUtils.ModulePosition modulePosition) {
		IAngleEncoder angleEncoder = EncoderFactory.createEncoder(swerveType, modulePosition);
		ControllableMotor steer = SteerFactory.createSteer(swerveType, modulePosition);
		ControllableMotor drive = DriveFactory.createDrive(swerveType, modulePosition);

		return new Module(
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

	public static Modules create(SwerveType swerveType) {
		return new Modules(
			swerveType.getLogPath(),
			createModule(swerveType, ModuleUtils.ModulePosition.FRONT_LEFT),
			createModule(swerveType, ModuleUtils.ModulePosition.FRONT_RIGHT),
			createModule(swerveType, ModuleUtils.ModulePosition.BACK_LEFT),
			createModule(swerveType, ModuleUtils.ModulePosition.BACK_RIGHT)
		);
	}

}
