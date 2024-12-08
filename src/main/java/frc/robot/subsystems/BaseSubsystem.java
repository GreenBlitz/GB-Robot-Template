package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;

public class BaseSubsystem<T> extends GBSubsystem {

	ControllableMotor motor;
	IRequest<T> request;

	public BaseSubsystem(ControllableMotor motor, String logPath, IRequest<T> request) {
		super(logPath);
		this.motor = motor;
		this.request = request;
	}

	public void resetPosition(Rotation2d position) {
		motor.resetPosition(position);
	}

	public void setBrake(Boolean brake) {
		motor.setBrake(brake);
	}

	public void applyRequest() {
		motor.applyRequest(request);
	}

}
