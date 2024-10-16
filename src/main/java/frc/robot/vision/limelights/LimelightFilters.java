package frc.robot.vision.limelights;

import edu.wpi.first.math.geometry.*;
import org.littletonrobotics.junction.Logger;

public class LimelightFilters {

	protected static boolean keepLimelightData(LimelightRawData limelightRawData, LimelightFiltersTolerances tolerances, String logpath) {
		boolean rollInTolerance = LimelightFilters.isRollInTolerance(limelightRawData, tolerances.rollTolerance());
		boolean pitchInTolerance = LimelightFilters.isPitchInTolerance(limelightRawData, tolerances.pitchTolerance());
		boolean robotOnGround = LimelightFilters.isRobotOnGround(limelightRawData, tolerances.robotToGroundToleranceMeters());
		if (!rollInTolerance) {
			Logger.recordOutput(logpath + "filteredBecauseBadRoll", limelightRawData.estimatedPose());
		}
		if (!pitchInTolerance) {
			Logger.recordOutput(logpath + "filteredBecauseBadPitch", limelightRawData.estimatedPose());
		}
		if (!robotOnGround) {
			Logger.recordOutput(logpath + "filteredBecauseRobotIsFuckingFlying", limelightRawData.estimatedPose());
		}

		return rollInTolerance && pitchInTolerance && robotOnGround;
	}

	protected static boolean isPitchInTolerance(LimelightRawData limelightRawData, Rotation2d pitchTolerance) {
		return Math.abs(limelightRawData.estimatedPose().getRotation().getY()) <= pitchTolerance.getRadians();
	}

	protected static boolean isRollInTolerance(LimelightRawData limelightRawData, Rotation2d rollTolerance) {
		return Math.abs(limelightRawData.estimatedPose().getRotation().getX()) <= rollTolerance.getRadians();
	}

	protected static boolean isRobotOnGround(LimelightRawData limelightRawData, double robotToGroundToleranceMeters) {
		return limelightRawData.estimatedPose().getZ() <= robotToGroundToleranceMeters;
	}

}
