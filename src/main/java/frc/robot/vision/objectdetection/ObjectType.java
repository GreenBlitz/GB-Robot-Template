package frc.robot.vision.objectdetection;

public enum ObjectType {

	ALGAE(0.41275, "algae\r"),
	CORAL(0.1143, "coral");

	private final double objectHeightMeters;
	private final String classificationEntryName;

	ObjectType(double objectHeightMeters, String classificationEntryName) {
		this.objectHeightMeters = objectHeightMeters;
		this.classificationEntryName = classificationEntryName;
	}

	public double getObjectHeightMeters() {
		return objectHeightMeters;
	}

	public String getClassificationEntryName() {
		return classificationEntryName;
	}

}
