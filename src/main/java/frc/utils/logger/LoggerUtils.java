package frc.utils.logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.utils.AngleUnit;
import org.littletonrobotics.junction.Logger;

public class LoggerUtils {

	public static void logRotation3d(Rotation3d toLog, String logPath, AngleUnit angleUnit) {
		Logger.recordOutput(logPath + "Roll", angleUnit.fromRotation2d(Rotation2d.fromRadians(toLog.getX())));
		Logger.recordOutput(logPath + "Pitch", angleUnit.fromRotation2d(Rotation2d.fromRadians(toLog.getY())));
		Logger.recordOutput(logPath + "Yaw", angleUnit.fromRotation2d(Rotation2d.fromRadians(toLog.getZ())));
	}

}
