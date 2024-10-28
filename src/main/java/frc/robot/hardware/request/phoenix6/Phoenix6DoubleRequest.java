package frc.robot.hardware.request.phoenix6;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.robot.hardware.request.IRequest;

import java.util.function.Consumer;

public class Phoenix6DoubleRequest implements IRequest<Double> {

	private final ControlRequest controlRequest;
	private final Consumer<Double> withSetPoint;
	private double setPoint;

	private Phoenix6DoubleRequest(ControlRequest controlRequest, Consumer<Double> withSetPoint) {
		this.withSetPoint = withSetPoint;
		this.controlRequest = controlRequest;
		this.setPoint = 0;
	}

	public Phoenix6DoubleRequest(VoltageOut voltageOut) {
		this(voltageOut, voltageOut::withOutput);
	}

	public Phoenix6DoubleRequest(TorqueCurrentFOC torqueCurrentFOC) {
		this(torqueCurrentFOC, torqueCurrentFOC::withOutput);
	}

	@Override
	public Phoenix6DoubleRequest withSetPoint(Double setPoint) {
		withSetPoint.accept(setPoint);
		this.setPoint = setPoint;
		return this;
	}
	
	@Override
	public Double getSetPoint() {
		return
	}
	
	public ControlRequest getControlRequest() {
		return controlRequest;
	}

}
