package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.utils.AngleUnit;
import org.littletonrobotics.junction.Logger;

public class Detection {
	
	public static NetworkTableEntry isValid = NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("tv");
	public static NetworkTableEntry tx = NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("tx");
	public static NetworkTableEntry ty = NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("ty");
	public static NetworkTableEntry name = NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("tdclass");
	
	public static double HEIGHT = 0.508;
	public static Rotation3d rotation3d = AngleUnit.DEGREES.toRotation3d(-8.06180374425555, -27.07784559039065, -22.52372569716833);
	
	public void log() {
		Rotation2d yaw = Rotation2d.fromDegrees(tx.getDouble(0));
		Rotation2d pitch = Rotation2d.fromDegrees(ty.getDouble(0)).unaryMinus();
		
		Translation2d trans = new Translation2d(yaw.getDegrees(), pitch.getDegrees());
		trans.rotateBy(Rotation2d.fromDegrees(10.612258493096334).unaryMinus());
		yaw = Rotation2d.fromDegrees(trans.getX());
		pitch = Rotation2d.fromDegrees(trans.getY());
		
		Logger.recordOutput("isvalid", isValid.getDouble(0));
		Logger.recordOutput("yaw", yaw);
		Logger.recordOutput("pitch", pitch);
		Logger.recordOutput("name", name.getString(""));
		double distanceX = (HEIGHT) / Math.tan(pitch.minus(Rotation2d.fromDegrees(-27.18966371065684)).getRadians());
		double distanceY = distanceX * Math.tan(yaw.minus(Rotation2d.fromDegrees(20.10328620400214)).getRadians());
		Logger.recordOutput("distancex", distanceX);
		Logger.recordOutput("distancey", distanceY);
		Logger.recordOutput("Final object pose", new Pose3d(new Translation3d(distanceX, distanceY, 0), new Rotation3d()));
	}
	
	
}
