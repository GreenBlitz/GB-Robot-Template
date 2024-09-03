package frc.robot.hardware.request.value;

public class SparkMaxValueRequest implements IValueRequest {

	private final int slot;
	private double setPoint;

	public SparkMaxValueRequest(double setPoint, int slot) {
		this.slot = slot;
		this.setPoint = setPoint;
	}

	@Override
	public void withSetPoint(double setPoint) {
		this.setPoint = setPoint;
	}

	public double getSetPoint() {
		return setPoint;
	}

	public int getSlot() {
		return slot;
	}

}
