package frc.robot.hardware.request;

import edu.wpi.first.math.geometry.Rotation2d;

public class SparkMaxControlRequest implements IControlRequest {

	private final int slot;
	private Rotation2d setPoint;

	public SparkMaxControlRequest(Rotation2d setPoint, int slot) {
		this.slot = slot;
		this.setPoint = setPoint;
	}

	@Override
	public void withSetPoint(Rotation2d setPoint) {
		this.setPoint = setPoint;
	}

	public Rotation2d getSetPoint() {
		return setPoint;
	}

	public int getSlot() {
		return slot;
	}

}
