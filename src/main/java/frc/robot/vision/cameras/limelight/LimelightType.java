package frc.robot.vision.cameras.limelight;

import edu.wpi.first.math.geometry.Translation2d;

public enum LimelightType {

	LIMELIGHT3(new Translation2d(0,0)),
	LIMELIGHT3GB(new Translation2d(0,0)),
	LIMELIGHT4(new Translation2d(0,0));

	private final Translation2d fieldOfView;


	LimelightType(Translation2d fieldOfViewY) {
		this.fieldOfView = fieldOfViewY;
	}

	public double getFieldOfViewX() {
		return fieldOfView.getX();
	}

	public double getFieldOfViewY(){
		return fieldOfView.getY();
	}


}
