package frc.robot.objectDetection;

import edu.wpi.first.networktables.NetworkTableInstance;

public class ObjectDetection {
	
	
	public static double getXAngle(){
		return NetworkTableInstance.getDefault().getTable(ObjectDetectionConstants.backLimelight).getEntry("tx").getDouble(0);
	}
	public static double getYAngle(){
		return NetworkTableInstance.getDefault().getTable(ObjectDetectionConstants.frontLimelight).getEntry("ty").getDouble(0);
	}
	
	
}
