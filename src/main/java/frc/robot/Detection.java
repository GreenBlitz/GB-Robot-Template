package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.utils.AngleUnit;
import org.littletonrobotics.junction.Logger;

public class Detection {
	
	public static NetworkTableEntry isValid = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tv");
	public static NetworkTableEntry tx = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tx");
	public static NetworkTableEntry ty = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("ty");
	public static NetworkTableEntry name = NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tdclass");
	
	public static double HEIGHT = 0.508;
	public static Rotation3d rotation3d = AngleUnit.DEGREES.toRotation3d(-8.06180374425555, -27.07784559039065, -22.52372569716833);
	
	public void log() {
		Rotation2d yaw = Rotation2d.fromDegrees(tx.getDouble(0));
		Rotation2d pitch = Rotation2d.fromDegrees(ty.getDouble(0));
		
		Logger.recordOutput("isvalid", isValid.getDouble(0));
		Logger.recordOutput("yaw", yaw);
		Logger.recordOutput("pitch", pitch);
		Logger.recordOutput("name", name.getString(""));
		double distanceY = (HEIGHT) / Math.tan(pitch.minus(Rotation2d.fromDegrees(-27.07784559039065)).getRadians());
		Logger.recordOutput("distancey", distanceY);
		Logger.recordOutput("distancey", distanceY * Math.tan(yaw.minus(Rotation2d.fromDegrees(-22.52372569716833)).getRadians()));
	}
	
	
}
