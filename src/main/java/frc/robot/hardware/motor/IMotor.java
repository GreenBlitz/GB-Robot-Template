package frc.robot.hardware.motor;

public interface IMotor {

	void setBrake(boolean brake);

	void stop();

	void setPower(double power);

	void updateInputs(MotorInputsAutoLogged motorInputs);

}
