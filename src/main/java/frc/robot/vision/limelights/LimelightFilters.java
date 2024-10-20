package frc.robot.vision.limelights;

import edu.wpi.first.math.geometry.*;
import frc.robot.constants.Field;
import org.littletonrobotics.junction.Logger;

public class LimelightFilters {

	protected static boolean keepLimelightData(LimelightRawData limelightRawData, LimelightFiltersTolerances tolerances, String logpath) {
		boolean rollInTolerance = LimelightFilters.isRollInTolerance(limelightRawData, tolerances.rollTolerance());
		boolean pitchInTolerance = LimelightFilters.isPitchInTolerance(limelightRawData, tolerances.pitchTolerance());
		boolean robotOnGround = LimelightFilters.isRobotOnGround(limelightRawData, tolerances.robotToGroundToleranceMeters());
		boolean isRobotInOfField = LimelightFilters.isRobotInField(limelightRawData);
		if (!rollInTolerance) {
			Logger.recordOutput(logpath + "filteredBecauseBadRoll", limelightRawData.estimatedPose());
		}
		if (!pitchInTolerance) {
			Logger.recordOutput(logpath + "filteredBecauseBadPitch", limelightRawData.estimatedPose());
		}
		if (!robotOnGround) {
			Logger.recordOutput(logpath + "filteredBecauseRobotIsFuckingFlying", limelightRawData.estimatedPose());
		}

		return rollInTolerance && pitchInTolerance && robotOnGround && isRobotInOfField;
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

	protected static boolean isRobotInField(LimelightRawData limelightRawData){
		Translation2d estimatedTranslation2d = limelightRawData.estimatedPose().toPose2d().getTranslation();
		return
				estimatedTranslation2d.getX() >= 0 &&
						estimatedTranslation2d.getY() >= 0 &&
						estimatedTranslation2d.getY() <= Field.WIDTH_METERS &&
						estimatedTranslation2d.getX() <= Field.LENGTH_METERS;


	}

}
