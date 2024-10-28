package frc.robot.hardware.request.phoenix6;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.request.IRequest;

import java.util.function.Consumer;

public class Phoenix6AngleRequest implements IRequest<Rotation2d> {

	private final ControlRequest controlRequest;
	private final Consumer<Rotation2d> withSetPoint;

	private Phoenix6AngleRequest(ControlRequest controlRequest, Consumer<Rotation2d> withSetPoint) {
		this.withSetPoint = withSetPoint;
		this.controlRequest = controlRequest;
	}

	public Phoenix6AngleRequest(PositionVoltage positionVoltage) {
		this(positionVoltage, setPoint -> positionVoltage.withPosition(setPoint.getRotations()));
	}

	public Phoenix6AngleRequest(VelocityVoltage velocityVoltage) {
		this(velocityVoltage, setPoint -> velocityVoltage.withVelocity(setPoint.getRotations()));
	}

	@Override
	public Phoenix6AngleRequest withSetPoint(Rotation2d setPoint) {
		withSetPoint.accept(setPoint);
		return this;
	}
	
	@Override
	public Rotation2d getSetPoint() {
		return
	}
	
	public ControlRequest getControlRequest() {
		return controlRequest;
	}

}
