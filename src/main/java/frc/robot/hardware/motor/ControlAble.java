package frc.robot.hardware.motor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.request.IRequest;
import frc.utils.calibration.sysid.SysIdCalibrator;

public interface ControlAble {

	SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo();

	void resetPosition(Rotation2d position);

	void applyDoubleRequest(IRequest<Double> request);

	void applyAngleRequest(IRequest<Rotation2d> request);

}
