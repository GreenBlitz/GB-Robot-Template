package frc.robot.poseestimator.helpers;

import org.littletonrobotics.junction.Logger;

public class PoseEstimatorLogging {
	
	public static void logVisionStandardDeviations(String logPath, double[] standardDeviations) {
		var newLogPath = logPath + "stdDevs/";
		Logger.recordOutput(newLogPath + "X/", standardDeviations[0]);
		Logger.recordOutput(newLogPath + "Y/", standardDeviations[1]);
		Logger.recordOutput(newLogPath + "Angle/", standardDeviations[2]);
	}
	
}
