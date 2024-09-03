package frc.robot.hardware.motor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.request.angle.IAngleRequest;
import frc.utils.calibration.sysid.SysIdCalibrator;

public interface ProfileAble {

	SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo();

	void resetAngle(Rotation2d angle);

	void setTargetProfiledVelocity(IAngleRequest profiledVelocityRequest);

	void setTargetProfiledAngle(IAngleRequest profiledAngleRequest);

	void updateInputs(PIDAbleInputsAutoLogged pidAbleInputs);

}
