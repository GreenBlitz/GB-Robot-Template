package frc.robot.subsystems.swerve.modules.steer.replay;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.ModuleInputsContainer;
import frc.robot.subsystems.swerve.modules.steer.ISteer;

public class EmptySteer implements ISteer {

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
