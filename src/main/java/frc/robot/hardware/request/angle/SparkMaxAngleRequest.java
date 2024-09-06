package frc.robot.hardware.request.angle;

import edu.wpi.first.math.geometry.Rotation2d;

public class SparkMaxAngleRequest implements IAngleRequest {

	private final int slot;
	private Rotation2d setPoint;

	public SparkMaxAngleRequest(Rotation2d setPoint, int slot) {
		this.slot = slot;
		this.setPoint = setPoint;
	}

	@Override
	public SparkMaxAngleRequest withSetPoint(Rotation2d setPoint) {
		this.setPoint = setPoint;
		return this;
	}

	public Rotation2d getSetPoint() {
		return setPoint;
	}

	public int getSlot() {
		return slot;
	}

}
