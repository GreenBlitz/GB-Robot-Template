package frc.robot.subsystems.swerve.factories.modules;

import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.subsystems.swerve.factories.modules.constants.ModuleSpecificConstantsFactory;
import frc.robot.subsystems.swerve.factories.modules.drive.DriveFactory;
import frc.robot.subsystems.swerve.factories.modules.encoder.EncoderFactory;
import frc.robot.subsystems.swerve.factories.modules.steer.SteerFactory;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.module.Modules;


public class ModulesFactory {

	private static Module createModule(String logPath, ModuleUtils.ModulePosition modulePosition) {
		IAngleEncoder angleEncoder = EncoderFactory.createEncoder(logPath, modulePosition);
		ControllableMotor steer = SteerFactory.createSteer(logPath, modulePosition);
		ControllableMotor drive = DriveFactory.createDrive(logPath, modulePosition);

		return new Module(
			ModuleSpecificConstantsFactory.create(logPath, modulePosition),
			angleEncoder,
			EncoderFactory.createSignals(angleEncoder),
			steer,
			SteerFactory.createRequests(),
			SteerFactory.createSignals(steer),
			drive,
			DriveFactory.createRequests(),
			DriveFactory.createSignals(drive)
		);
	}

	public static Modules create(String logPath) {
		return new Modules(
			logPath,
			createModule(logPath, ModuleUtils.ModulePosition.FRONT_LEFT),
			createModule(logPath, ModuleUtils.ModulePosition.FRONT_RIGHT),
			createModule(logPath, ModuleUtils.ModulePosition.BACK_LEFT),
			createModule(logPath, ModuleUtils.ModulePosition.BACK_RIGHT)
		);
	}

}
