package frc.robot.vision.objectdetection;

public enum ObjectType {

	ALGAE("algae\r", 0.41275),
	CORAL("coral", 0.1143);

	private final String nameEntryValue;
	private final double objectHeightMeters;

	ObjectType(String nameEntryValue, double objectHeightMeters) {
		this.nameEntryValue = nameEntryValue;
		this.objectHeightMeters = objectHeightMeters;
	}

	public String getNameEntryValue() {
		return nameEntryValue;
	}

	public double getObjectHeightMeters() {
		return objectHeightMeters;
	}

}
