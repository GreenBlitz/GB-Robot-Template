package frc.robot.hardware.motor;

import frc.robot.hardware.IDevice;

public interface IMotor extends IDevice {

    void setBrake(boolean brake);

    void stop();

    void setPower(double power);

}
