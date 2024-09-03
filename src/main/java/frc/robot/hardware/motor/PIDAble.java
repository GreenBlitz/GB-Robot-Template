package frc.robot.hardware.motor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.request.angle.IAngleRequest;
import frc.robot.hardware.request.value.IValueRequest;
import frc.utils.calibration.sysid.SysIdCalibrator;

public interface PIDAble {

	SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo();

	void resetAngle(Rotation2d angle);

	void setVoltage(IValueRequest voltageRequest);

	void setTargetVelocity(IAngleRequest velocityRequest);

	void setTargetAngle(IAngleRequest angleRequest);

	void updateInputs(PIDAbleInputsAutoLogged pidAbleInputs);

}
