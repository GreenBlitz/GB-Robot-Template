package frc.robot.hardware.request.value;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;

import java.util.function.Consumer;

public class TalonFXValueRequest implements IValueRequest {

	private final Consumer<Double> withSetPoint;
	private final ControlRequest controlRequest;

	public TalonFXValueRequest(ControlRequest controlRequest, Consumer<Double> withSetPoint) {
		this.withSetPoint = withSetPoint;
		this.controlRequest = controlRequest;
	}

	public TalonFXValueRequest(VoltageOut voltageOut) {
		this(voltageOut, voltageOut::withOutput);
	}

	public TalonFXValueRequest(TorqueCurrentFOC torqueCurrentFOC) {
		this(torqueCurrentFOC, torqueCurrentFOC::withOutput);
	}

	@Override
	public void withSetPoint(double setPoint) {
		withSetPoint.accept(setPoint);
	}

	public ControlRequest getControlRequest() {
		return controlRequest;
	}

}
