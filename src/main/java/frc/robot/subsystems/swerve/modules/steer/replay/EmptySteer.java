package frc.robot.subsystems.swerve.modules.steer.replay;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.swerve.modules.ModuleInputsContainer;
import frc.robot.subsystems.swerve.modules.steer.ISteer;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class EmptySteer implements ISteer {

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysIdConfigInfo() {
		return new SysIdCalibrator.SysIdConfigInfo(new SysIdRoutine.Config(), false);
	}

	@Override
	public void setBrake(boolean brake) {}

	@Override
	public void resetToAngle(Rotation2d angle) {}

	@Override
	public void stop() {}

	@Override
	public void setVoltage(double voltage) {}

	@Override
	public void setTargetAngle(Rotation2d angle) {}

	@Override
	public void updateInputs(ModuleInputsContainer inputs) {}

}
