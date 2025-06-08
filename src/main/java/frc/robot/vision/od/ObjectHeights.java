package frc.robot.vision.od;

public enum ObjectHeights {

	ALGAE(0.41275),
	CORAL(0.1143);

	public final double objectHeightMeters;

	ObjectHeights(double objectHeightMeters) {
		this.objectHeightMeters = objectHeightMeters;
	}

}
