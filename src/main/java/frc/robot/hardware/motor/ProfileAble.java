package frc.robot.hardware.motor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.request.IRequest;
import frc.utils.calibration.sysid.SysIdCalibrator;

public interface ProfileAble {

    SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo();
    void resetAngle(Rotation2d angle);
    void setTargetProfiledAngle(IRequest request);
    void setTargetProfiledVelocity(IRequest request);
    void updateInputs(PIDAbleInputsAutoLogged inputs);

}
