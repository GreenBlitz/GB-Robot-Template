package frc.robot.hardware.motor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.request.IRequest;
import frc.utils.calibration.sysid.SysIdCalibrator;

public interface PIDAble {

    SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo();
    void resetAngle(Rotation2d angle);
    void setTargetAngle(IRequest request);
    void setTargetVelocity(IRequest request);
    void updateInputs(PIDAbleInputsAutoLogged inputs);

}
