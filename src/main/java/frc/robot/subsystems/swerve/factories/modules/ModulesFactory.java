package frc.robot.subsystems.swerve.factories.modules;

import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.subsystems.swerve.factories.modules.constants.SimulationModuleConstants;
import frc.robot.subsystems.swerve.factories.modules.drive.DriveFactory;
import frc.robot.subsystems.swerve.factories.modules.encoder.SimulationEncoderBuilder;
import frc.robot.subsystems.swerve.factories.modules.steer.SteerFactory;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.robot.subsystems.swerve.module.Modules;

public class ModulesFactory {

	private static Module createModule(String logPath, ModuleUtil.ModulePosition modulePosition) {
		IAngleEncoder angleEncoder = SimulationEncoderBuilder.buildEncoder(logPath);
		ControllableMotor steer = SteerFactory.createSteer(logPath, modulePosition);
		ControllableMotor drive = DriveFactory.createDrive(logPath, modulePosition);

		return new Module(
			SimulationModuleConstants.getModuleSpecificConstants(logPath, modulePosition),
			angleEncoder,
			SimulationEncoderBuilder.buildSignals(),
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
			createModule(logPath, ModuleUtil.ModulePosition.FRONT_LEFT),
			createModule(logPath, ModuleUtil.ModulePosition.FRONT_RIGHT),
			createModule(logPath, ModuleUtil.ModulePosition.BACK_LEFT),
			createModule(logPath, ModuleUtil.ModulePosition.BACK_RIGHT)
		);
	}

}
