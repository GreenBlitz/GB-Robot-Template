package frc.robot.hardware.motor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.request.IControlRequest;
import frc.utils.calibration.sysid.SysIdCalibrator;

public interface PIDAble {

	SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo();

	void resetAngle(Rotation2d angle);

	void setVoltage(double voltage);

	void setTargetVelocity(IControlRequest controlRequest);

	void setTargetAngle(IControlRequest controlRequest);

	void updateInputs(PIDAbleInputsAutoLogged pidAbleInputs);

}
