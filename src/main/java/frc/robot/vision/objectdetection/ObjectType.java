package frc.robot.vision.objectdetection;

public enum ObjectType {

	ALGAE(0.41275, "algae\r"),
	CORAL(0.1143, "coral");

	public final double objectHeightMeters;
	public final String classificationEntryName;

	ObjectType(double objectHeightMeters, String classificationEntryName) {
		this.objectHeightMeters = objectHeightMeters;
		this.classificationEntryName = classificationEntryName;
	}

}
