package frc.robot.hardware.request.srx;

import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.hardware.request.IRequest;

public class DoubleSRXRequest implements IRequest<Double> {

	private final ControlMode controlMode;
	private double setPoint;

	public DoubleSRXRequest(ControlMode controlMode) {
		this.controlMode = controlMode;
		this.setPoint = 0;
	}

	@Override
	public IRequest<Double> withSetPoint(Double setPoint) {
		this.setPoint = setPoint;
		return this;
	}

	public ControlMode getControlMode() {
		return controlMode;
	}

	public double getSetPoint() {
		return setPoint;
	}

}
