package frc.robot.subsystems.swerve.modules.drive.replay;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.swerve.modules.ModuleInputsContainer;
import frc.robot.subsystems.swerve.modules.drive.IDrive;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class EmptyDrive implements IDrive {

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysIdConfigInfo() {
		return new SysIdCalibrator.SysIdConfigInfo(new SysIdRoutine.Config(), false);
	}

	@Override
	public void setBrake(boolean brake) {}

	@Override
	public void stop() {}

	@Override
	public void setVoltage(double voltage) {}

	@Override
	public void setTargetVelocity(Rotation2d velocityPerSecond) {}

	@Override
	public void updateInputs(ModuleInputsContainer inputs) {}

}
