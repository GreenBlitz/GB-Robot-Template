package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.calibration.sysid.SysIdCalibrator;

import java.util.function.Supplier;

public class ModulesCommandsBuilder {

	private final Modules modules;
	private final SysIdCalibrator steerCalibrator;
	private final SysIdCalibrator driveCalibrator;
	private GBSubsystem[] subsystemsToRequire;


	public ModulesCommandsBuilder(Modules modules) {
		this.modules = modules;
		this.steerCalibrator = new SysIdCalibrator(
			modules.getModule(ModuleUtils.ModulePosition.FRONT_LEFT).getSteerSysIdConfigInfo(),
			modules,
			modules.getModule(ModuleUtils.ModulePosition.FRONT_LEFT)::setSteerVoltage
		);
		this.driveCalibrator = new SysIdCalibrator(
			modules.getModule(ModuleUtils.ModulePosition.FRONT_LEFT).getDriveSysIdConfigInfo(),
			modules,
			modules::setDrivesVoltage
		);
	}

	public ModulesCommandsBuilder withSubsystemsToRequire(GBSubsystem... subsystemsToRequire) {
		this.subsystemsToRequire = subsystemsToRequire;
		return this;
	}

	private Command toModulesCommand(Command command) {
		command.addRequirements(subsystemsToRequire);
		command.addRequirements(modules);
		return command;
	}

	public Command steerCalibration(boolean isQuasistatic, SysIdRoutine.Direction direction) {
		return toModulesCommand(steerCalibrator.getSysIdCommand(isQuasistatic, direction)).withName("Steer calibration");
	}

	public Command driveCalibration(boolean isQuasistatic, SysIdRoutine.Direction direction) {
		Command sysIdCommand = driveCalibrator.getSysIdCommand(isQuasistatic, direction);
		sysIdCommand.getRequirements().clear();

		return toModulesCommand(new SequentialCommandGroup(
			pointWheels(new Rotation2d(), false).until(
				() -> modules.isSteersAtTargetPositions(
					ModuleConstants.CALIBRATION_MODULE_ANGLE_TOLERANCE,
					ModuleConstants.CALIBRATION_MODULE_ANGLE_VELOCITY_PER_SECOND_DEADBAND
				)
			),
			new ParallelDeadlineGroup(sysIdCommand, pointWheels(new Rotation2d(), false))
		)).withName("Drive calibration");
	}


	public Command pointWheels(Rotation2d targetSteerPosition, boolean optimize) {
		return toModulesCommand(new RunCommand(() -> modules.pointWheels(targetSteerPosition, optimize))).withName("Point wheels to: " + targetSteerPosition);
	}

	public Command pointWheelsInCircle() {
		return toModulesCommand(new RunCommand(modules::pointWheelsInCircle)).withName("Point wheels in circle");
	}

	public Command pointWheelsInX() {
		return toModulesCommand(new RunCommand(() -> modules.pointWheelsInX(ModuleConstants.DEFAULT_IS_CLOSE_LOOP)).withName("Point wheels in X"));
	}

	public Command setTargetStates(Supplier<SwerveModuleState[]> statesSupplier, boolean isClosedLoop) {
		return toModulesCommand(new RunCommand(() -> modules.setTargetStates(statesSupplier.get(), isClosedLoop)).withName("Set states by supplier"));
	}

}
