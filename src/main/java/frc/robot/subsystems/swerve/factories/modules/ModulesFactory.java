package frc.robot.subsystems.swerve.factories.modules;

import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IAngleEncoder;
import frc.robot.subsystems.swerve.factories.modules.constants.ModuleSpecificConstantsFactory;
import frc.robot.subsystems.swerve.factories.modules.drive.DriveFactory;
import frc.robot.subsystems.swerve.factories.modules.encoder.EncoderFactory;
import frc.robot.subsystems.swerve.factories.modules.steer.SteerFactory;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.robot.subsystems.swerve.module.Modules;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class ModulesFactory {

	private static Module createModule(String logPath, ModuleUtil.ModulePosition modulePosition, SwerveDriveSimulation swerveDriveSimulation) {
		IAngleEncoder angleEncoder = EncoderFactory.createEncoder(logPath, modulePosition);
		ControllableMotor steer = SteerFactory.createSteer(logPath, modulePosition, swerveDriveSimulation);
		ControllableMotor drive = DriveFactory.createDrive(logPath, modulePosition, swerveDriveSimulation);

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

	public static Modules create(String logPath, SwerveDriveSimulation swerveDriveSimulation) {
		return new Modules(
			logPath,
			createModule(logPath, ModuleUtil.ModulePosition.FRONT_LEFT, swerveDriveSimulation),
			createModule(logPath, ModuleUtil.ModulePosition.FRONT_RIGHT, swerveDriveSimulation),
			createModule(logPath, ModuleUtil.ModulePosition.BACK_LEFT, swerveDriveSimulation),
			createModule(logPath, ModuleUtil.ModulePosition.BACK_RIGHT, swerveDriveSimulation)
		);
	}

}
