package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class ModulesCommandsBuilder {

	private final Modules modules;
	private final SysIdCalibrator steerCalibrator;
	private final SysIdCalibrator driveCalibrator;

	public ModulesCommandsBuilder(Modules modules) {
		this.modules = modules;
		this.steerCalibrator = new SysIdCalibrator(
			modules.getModule(ModuleUtils.ModulePosition.FRONT_LEFT).getSteerSysIdConfigInfo(),
			modules,
			voltage -> modules.getModule(ModuleUtils.ModulePosition.FRONT_LEFT).setSteerVoltage(voltage)
		);
		this.driveCalibrator = new SysIdCalibrator(
			modules.getModule(ModuleUtils.ModulePosition.FRONT_LEFT).getDriveSysIdConfigInfo(),
			modules,
			modules::setDrivesVoltage
		);
	}

	public Command steerCalibration(boolean isQuasistatic, SysIdRoutine.Direction direction) {
		return steerCalibrator.getSysIdCommand(isQuasistatic, direction).withName("Steer calibration");
	}

	public Command driveCalibration(boolean isQuasistatic, SysIdRoutine.Direction direction) {
		Command sysIdCommand = driveCalibrator.getSysIdCommand(isQuasistatic, direction);
		sysIdCommand.getRequirements().clear();

		return new SequentialCommandGroup(
			pointWheels(new Rotation2d(), false).until(
				() -> modules.isAtTargetAngles(
					ModuleConstants.CALIBRATION_MODULE_ANGLE_TOLERANCE,
					ModuleConstants.CALIBRATION_MODULE_ANGLE_VELOCITY_PER_SECOND_DEADBAND
				)
			),
			new ParallelDeadlineGroup(sysIdCommand, pointWheels(new Rotation2d(), false))
		).withName("Drive calibration");
	}


	public Command pointWheels(Rotation2d wheelsAngle, boolean optimize) {
		return new RunCommand(() -> modules.pointWheels(wheelsAngle, optimize), modules).withName("Point wheels to: " + wheelsAngle);
	}

	public Command pointWheelsInCircle() {
		return new RunCommand(modules::pointWheelsInCircle, modules).withName("Point wheels in circle");
	}

	public Command pointWheelsInX() {
		return new RunCommand(() -> modules.pointWheelsInX(ModuleConstants.DEFAULT_IS_CLOSE_LOOP), modules).withName("Point wheels in X");
	}

}
