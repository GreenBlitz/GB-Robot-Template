package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.ControlRequest;
import frc.robot.hardware.interfaces.IRequest;

import java.util.function.Consumer;

public class Phoenix6Request<T> implements IRequest<T> {

	private final ControlRequest controlRequest;
	private final Consumer<T> setSetPoint;
	private T setPoint;

	Phoenix6Request(T defaultSetPoint, ControlRequest controlRequest, Consumer<T> setSetPoint) {
		this.setPoint = defaultSetPoint;
		this.controlRequest = controlRequest;
		this.setSetPoint = setSetPoint;
	}

	@Override
	public Phoenix6Request<T> withSetPoint(T setPoint) {
		setSetPoint.accept(setPoint);
		this.setPoint = setPoint;
		return this;
	}

	@Override
	public T getSetPoint() {
		return setPoint;
	}

	public ControlRequest getControlRequest() {
		return controlRequest;
	}

}
