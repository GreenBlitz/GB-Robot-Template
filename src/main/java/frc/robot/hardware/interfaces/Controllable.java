package frc.robot.hardware.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.calibration.sysid.SysIdCalibrator;

public interface Controllable {

	SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo();

	void resetPosition(Rotation2d position);

	void applyRequest(IRequest<?> request);

}
