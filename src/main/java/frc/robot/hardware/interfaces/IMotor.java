package frc.robot.hardware.interfaces;

public interface IMotor extends IDevice {

	void updateSimulation();

	void setBrake(boolean brake);

}
