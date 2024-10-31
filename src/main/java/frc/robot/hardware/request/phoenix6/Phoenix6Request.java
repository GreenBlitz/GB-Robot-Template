package frc.robot.hardware.request.phoenix6;

import com.ctre.phoenix6.controls.ControlRequest;
import frc.robot.hardware.request.IRequest;

import java.util.function.Consumer;

public class Phoenix6Request<T> implements IRequest<T> {

	private final ControlRequest controlRequest;
	private final Consumer<T> withSetPoint;

	Phoenix6Request(ControlRequest controlRequest, Consumer<T> withSetPoint) {
		this.withSetPoint = withSetPoint;
		this.controlRequest = controlRequest;
	}

	@Override
	public Phoenix6Request<T> withSetPoint(T setPoint) {
		withSetPoint.accept(setPoint);
		return this;
	}

	public ControlRequest getControlRequest() {
		return controlRequest;
	}

}
