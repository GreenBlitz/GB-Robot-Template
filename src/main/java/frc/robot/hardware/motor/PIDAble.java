package frc.robot.hardware.motor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.request.angle.IAngleRequest;
import frc.robot.hardware.request.value.IValueRequest;
import frc.utils.calibration.sysid.SysIdCalibrator;

public interface PIDAble {

	SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo();

	void resetPosition(Rotation2d position);

	void setVoltage(IValueRequest voltageRequest);

	void setTargetVelocity(IAngleRequest velocityRequest);

	void setTargetPosition(IAngleRequest positionRequest);

	void updateInputs(PIDAbleInputsAutoLogged pidAbleInputs);

}
