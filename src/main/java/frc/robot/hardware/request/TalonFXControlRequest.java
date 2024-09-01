package frc.robot.hardware.request;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.Consumer;

public class TalonFXControlRequest implements IControlRequest {

	private final Consumer<Rotation2d> withSetPoint;
	private final ControlRequest controlRequest;

	public TalonFXControlRequest(ControlRequest controlRequest, Consumer<Rotation2d> withSetPoint) {
		this.controlRequest = controlRequest;
		this.withSetPoint = withSetPoint;
	}

	@Override
	public void withSetPoint(Rotation2d setPoint) {
		withSetPoint.accept(setPoint);
	}

	public ControlRequest getControlRequest() {
		return controlRequest;
	}

}
