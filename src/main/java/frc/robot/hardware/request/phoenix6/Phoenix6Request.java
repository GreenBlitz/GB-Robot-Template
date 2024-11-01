package frc.robot.hardware.request.phoenix6;

import com.ctre.phoenix6.controls.ControlRequest;
import frc.robot.hardware.request.IRequest;

import java.util.function.Consumer;

public class Phoenix6Request<T> implements IRequest<T> {

	private final ControlRequest controlRequest;
	private final Consumer<T> withSetPoint;
	private T setPoint;

	Phoenix6Request(T defaultSetPoint, ControlRequest controlRequest, Consumer<T> withSetPoint) {
		this.withSetPoint = withSetPoint;
		this.controlRequest = controlRequest;
		this.setPoint = defaultSetPoint;
	}

	@Override
	public Phoenix6Request<T> withSetPoint(T setPoint) {
		withSetPoint.accept(setPoint);
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
