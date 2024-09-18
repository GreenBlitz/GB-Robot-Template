package frc.robot.hardware.motor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.request.IRequest;

public interface ControlAble {

//    SysIdCalibrator.SysId getSysidConfigInfo(); //TODO

    void resetPosition(Rotation2d position);

    void applyDoubleRequest(IRequest<Double> request);

    void applyAngleRequest(IRequest<Rotation2d> request);

}
