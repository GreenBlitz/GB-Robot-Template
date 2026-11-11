package frc.robot.hardware.interfaces;

import frc.utils.calibration.sysid.SysIdCalibrator;
import org.wpilib.math.geometry.Rotation2d;

public interface Controllable {

  SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo();

  void resetPosition(Rotation2d position);

  void applyRequest(IRequest<?> request);
}
