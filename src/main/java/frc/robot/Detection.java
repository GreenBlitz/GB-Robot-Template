package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.utils.AngleUnit;
import org.littletonrobotics.junction.Logger;

public class Detection {
	
	public static NetworkTableEntry isValid = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tv");
	public static NetworkTableEntry xAngle = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tx");
	public static NetworkTableEntry yAngle = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("ty");
	public static NetworkTableEntry name = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tdclass");
	
	public static double HEIGHT = 0.508;
	public static Rotation3d rotation3d = AngleUnit.DEGREES.toRotation3d(-8.06180374425555, -27.07784559039065, -22.52372569716833);
	
	public void log() {
		Logger.recordOutput("isvalid", isValid.getDouble(0));
		Logger.recordOutput("xAngle", xAngle.getDouble(0));
		Logger.recordOutput("yAngle", yAngle.getDouble(0));
		Logger.recordOutput("name", name.getString("yes"));
//		Logger.recordOutput("distancex", (HEIGHT) / Math.tan(Rotation2d.fromDegrees(xAngle.getDouble(0) + -8.06180374425555).getRadians()));
		Logger.recordOutput("distancey", (HEIGHT) / Math.tan(Rotation2d.fromDegrees(yAngle.getDouble(0) + -27.07784559039065).getRadians()));
	}


}
