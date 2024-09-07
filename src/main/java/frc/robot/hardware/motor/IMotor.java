package frc.robot.hardware.motor;

import frc.robot.hardware.signal.InputSignal;

public interface IMotor {

	void setBrake(boolean brake);

	void stop();

	void setPower(double power);

	void fetchSignals(InputSignal... signals);

}
