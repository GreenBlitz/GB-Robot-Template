package frc.robot.hardware.interfaces;

import org.wpilib.math.geometry.Angle2d;
import frc.utils.calibration.sysid.SysIdCalibrator;

public interface Controllable {

	SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo();

	void resetPosition(Angle2d position);

	void applyRequest(IRequest<?> request);

}
