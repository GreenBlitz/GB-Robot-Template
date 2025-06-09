package frc.robot.vision.od;

public enum ObjectType {

	ALGAE(0.41275),
	CORAL(0.1143);

	public final double objectHeightMeters;

	ObjectType(double objectHeightMeters) {
		this.objectHeightMeters = objectHeightMeters;
	}

}
