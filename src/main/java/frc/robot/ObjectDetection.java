package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;

public class ObjectDetection {
	
	
	public static double get(){
		return NetworkTableInstance.getDefault().getTable("limelight-back").getEntry("tx").getDouble(0);
	}
	
	
}
