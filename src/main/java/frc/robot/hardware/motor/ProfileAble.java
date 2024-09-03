package frc.robot.hardware.motor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.request.IControlRequest;
import frc.utils.calibration.sysid.SysIdCalibrator;

public interface ProfileAble {

	SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo();

	void resetAngle(Rotation2d angle);

	void setTargetProfiledVelocity(IControlRequest controlRequest);

	void setTargetProfiledAngle(IControlRequest controlRequest);

	void updateInputs(PIDAbleInputsAutoLogged pidAbleInputs);

}
