package frc.robot.poseestimator.helpers;

import org.littletonrobotics.junction.Logger;

public class PoseEstimatorLogging {

	public static void logStandardDeviations(String logPath, StandardDeviations2D standardDeviations) {
		var newLogPath = logPath + "stdDevs/";
		Logger.recordOutput(newLogPath + "X/", standardDeviations.xStandardDeviations());
		Logger.recordOutput(newLogPath + "Y/", standardDeviations.yStandardDeviations());
		Logger.recordOutput(newLogPath + "Angle/", standardDeviations.angleStandardDeviations());
	}

}
