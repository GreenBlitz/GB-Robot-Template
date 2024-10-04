package frc.robot.hardware.request.srx;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.request.IRequest;

public class AngleSRXRequest implements IRequest<Rotation2d> {

	private final ControlMode controlMode;
	private final int pidSlot;
	private Rotation2d setPoint;

	public AngleSRXRequest(ControlMode controlMode, int pidSlot) {
		this.setPoint = new Rotation2d();
		this.pidSlot = pidSlot;
		this.controlMode = controlMode;
	}

	@Override
	public IRequest<Rotation2d> withSetPoint(Rotation2d setPoint) {
		this.setPoint = setPoint;
		return this;
	}

	public ControlMode getControlMode() {
		return controlMode;
	}

	public int getPidSlot() {
		return pidSlot;
	}

	public Rotation2d getSetPoint() {
		return setPoint;
	}

}
