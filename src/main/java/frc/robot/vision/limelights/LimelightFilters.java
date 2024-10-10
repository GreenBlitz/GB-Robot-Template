package frc.robot.vision.limelights;

import edu.wpi.first.math.geometry.*;
import org.littletonrobotics.junction.Logger;

public class LimelightFilters {

	protected static boolean keepLimelightData(
		LimelightRawData limelightRawData,
		Pose2d currentEstimatedPose,
		double aprilTagHeightMeters,
		LimelightFiltersTolerances tolerances
	) {
		return LimelightFilters.isAprilTagInProperHeight(limelightRawData, tolerances.aprilTagHeightToleranceMeters(), aprilTagHeightMeters)
			&& LimelightFilters.isLimelightOutputInTolerance(
				limelightRawData,
				currentEstimatedPose,
				tolerances.normalizedPositionTolerance(),
				tolerances.normalizedRotationTolerance()
			)
			&& LimelightFilters.isRollInTolerance(limelightRawData, tolerances.rollTolerance())
			&& LimelightFilters.isPitchInTolerance(limelightRawData, tolerances.pitchTolerance())
			&& LimelightFilters.isRobotOnGround(limelightRawData, tolerances.robotToGroundToleranceMeters());
	}

	protected static boolean isLimelightOutputInTolerance(
		LimelightRawData limelightRawData,
		Pose2d estimatedPose,
		double normalizedPositionTolerance,
		double normalizedRotationTolerance
	) {
		Pose3d limelightPosition = limelightRawData.estimatedPose();
		Pose3d estimatedPose3d = new Pose3d(
			estimatedPose.getX(),
			estimatedPose.getY(),
			0,
			new Rotation3d(0, 0, estimatedPose.getRotation().getRadians())
		);
		Transform3d transformDifference = limelightPosition.minus(estimatedPose3d);
		Rotation3d rotationDifference = limelightPosition.getRotation().minus(estimatedPose3d.getRotation());
		boolean output = transformDifference.getTranslation().getNorm() <= normalizedPositionTolerance
			&& getRotationNorm(rotationDifference) <= normalizedRotationTolerance;
		if (output) {
			Logger.recordOutput("filtered because output tolarance: ", limelightRawData.estimatedPose());
		}
		return output;
	}

	private static double getRotationNorm(Rotation3d angle) {
		return Math.sqrt(Math.pow(angle.getX(), 2) + Math.pow(angle.getY(), 2) + Math.pow(angle.getZ(), 2));
	}

	protected static boolean isPitchInTolerance(LimelightRawData limelightRawData, Rotation2d pitchTolerance) {
		boolean output = Math.abs(limelightRawData.estimatedPose().getRotation().getY()) <= pitchTolerance.getRadians();
		if (output) {
			Logger.recordOutput("filtered because bad pitch: ", limelightRawData.estimatedPose());
		}
		return output;
	}

	protected static boolean isRollInTolerance(LimelightRawData limelightRawData, Rotation2d rollTolerance) {
		boolean output = Math.abs(limelightRawData.estimatedPose().getRotation().getX()) <= rollTolerance.getRadians();
		if (output) {
			Logger.recordOutput("filtered because bad roll: ", limelightRawData.estimatedPose());
		}
		return output;
	}

	protected static boolean
		isAprilTagInProperHeight(LimelightRawData limelightRawData, double aprilTagHeightToleranceMeters, double aprilTagHeightMeters) {
		double aprilTagHeightConfidence = Math.abs(limelightRawData.aprilTagHeight() - aprilTagHeightMeters);
		boolean output = aprilTagHeightConfidence <= aprilTagHeightToleranceMeters;
		if (!output) {
			Logger.recordOutput("filtered because height: ", limelightRawData.estimatedPose());
		}
		return output;
	}

	protected static boolean isRobotOnGround(LimelightRawData limelightRawData, double robotToGroundToleranceMeters) {
		boolean output = limelightRawData.estimatedPose().getZ() <= robotToGroundToleranceMeters;
		if (output) {
			Logger.recordOutput("filtered because robot is flying: ", limelightRawData.estimatedPose());
		}
		return output;
	}

}
