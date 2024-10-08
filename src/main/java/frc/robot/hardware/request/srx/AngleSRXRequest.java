package frc.robot.hardware.request.srx;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.request.IRequest;

public class AngleSRXRequest implements IRequest<Rotation2d> {

	private Rotation2d setPoint;
	private final int pidSlot;
	private final ControlMode controlMode;

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

	public Rotation2d getSetPoint() {
		return setPoint;
	}

	public int getPidSlot() {
		return pidSlot;
	}

	public ControlMode getControlMode() {
		return controlMode;
	}

}
