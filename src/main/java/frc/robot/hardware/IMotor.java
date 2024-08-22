package frc.robot.hardware;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.calibration.sysid.SysIdCalibrator;

public interface IMotor {

    SysIdCalibrator.SysIdConfigInfo getSysIdConfigInfo();

    void setBrake(boolean brake);

    void resetAngle(Rotation2d angle);

    void stop();

    void setVoltage(double voltage);

    void setTargetVelocity(CloseLoopControl velocityControl);

    void setTargetAngle(CloseLoopControl positionControl);

    void updateInputs(MotorInputs motorInputs);

}
