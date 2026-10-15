package frc.utils.limelight;

public class LimelightHelpersAdditions {

	public static boolean getIsConnected(String limelightName) {
		return LimelightHelpers.getLimelightNTTable(limelightName).containsKey("getpipe");
	}

	public static double getTemperatureCelsius(String limelightName) {
		return LimelightHelpers.getLimelightNTTable(limelightName).getEntry("hw").getDoubleArray(new double[] {0})[0];
	}

}
