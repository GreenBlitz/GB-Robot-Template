package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.*;
import frc.robot.constants.Field;
import java.util.List;
import java.util.Optional;

public class PoseEstimationMath {

	public static Optional<Rotation2d> calculateAngleAverage(List<Rotation2d> estimatedHeadings) {
		double summedXComponent = 0;
		double summedYComponent = 0;
		for (Rotation2d heading : estimatedHeadings) {
			summedXComponent += heading.getCos();
			summedYComponent += heading.getSin();
		}
		if (summedXComponent == 0 || summedYComponent == 0 || estimatedHeadings.isEmpty()) {
			return Optional.empty();
		}
		summedXComponent /= estimatedHeadings.size();
		summedYComponent /= estimatedHeadings.size();
		return Optional.of(new Rotation2d(Math.atan2(summedYComponent, summedXComponent)));
	}

	public static double distanceBetweenPosesMeters(Pose2d first, Pose2d second) {
		return first.minus(second).getTranslation().getNorm();
	}

	public static double distanceBetweenPosesMeters(Pose3d first, Pose3d second) {
		return first.minus(second).getTranslation().getNorm();
	}

}
