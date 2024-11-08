package frc.robot.hardware.interfaces;

public interface IMotor extends IDevice {

	void setBrake(boolean brake);

	void stop();

	void setPower(double power);

}