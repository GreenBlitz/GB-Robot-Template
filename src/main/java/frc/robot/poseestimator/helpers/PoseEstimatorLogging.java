package frc.robot.poseestimator.helpers;

import org.littletonrobotics.junction.Logger;

public class PoseEstimatorLogging {

	public static void logStandardDeviations(String logPath, StandardDeviations2d standardDeviations) {
		var newLogPath = logPath + "stdDevs/";
		Logger.recordOutput(newLogPath + "X/", standardDeviations.getXStandardDeviationsMeters());
		Logger.recordOutput(newLogPath + "Y/", standardDeviations.getYStandardDeviationsMeters());
		Logger.recordOutput(newLogPath + "Angle/", standardDeviations.getAngleStandardDeviations());
	}

}
