package frc.robot.hardware.interfaces;

import frc.utils.calibration.sysid.SysIdCalibrator;
import frc.utils.costomUnits.Angle2d;

public interface Controllable {

	SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo();

	void resetPosition(Angle2d position);

	void applyRequest(IRequest<?> request);

}
