package frc.robot.subsystems.swerve.factories.modules;

<<<<<<< HEAD
import frc.robot.Robot;
=======
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IAngleEncoder;
>>>>>>> core-swerve
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
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;


public class ModulesFactory {

<<<<<<< HEAD
	//@formatter:off
	private static Module createModule(SwerveType swerveType, ModuleUtils.ModulePosition modulePosition, SwerveModuleSimulation moduleSimulation) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL ->
				new HardwareModule(
					ModuleConstantsFactory.create(swerveType, modulePosition),
					EncoderFactory.create(swerveType, modulePosition),
					SteerFactory.create(swerveType, modulePosition),
					DriveFactory.create(swerveType, modulePosition)
				);
			case SIMULATION -> new MapleModule(
				ModuleConstantsFactory.create(swerveType, modulePosition),
				SimulationModuleGenerator.generateMapleModuleConstants(),
				moduleSimulation
			);
		};
=======
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
>>>>>>> core-swerve
	}
	//@formatter:on

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
