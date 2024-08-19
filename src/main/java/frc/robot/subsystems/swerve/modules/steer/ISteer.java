package frc.robot.subsystems.swerve.modules.steer;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.ModuleInputsContainer;
import frc.utils.calibration.sysid.SysIdCalibrator;

public interface ISteer {

	SysIdCalibrator.SysIdConfigInfo getSysIdConfigInfo();

	void setBrake(boolean brake);

	void resetToAngle(Rotation2d angle);

	void stop();

	void setVoltage(double voltage);

	void setTargetAngle(Rotation2d angle);

	void updateInputs(SteerInputsAutoLogged inputs);

}
