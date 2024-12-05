package frc.robot.vision.multivisionsources;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.GyroAngleValues;
import frc.robot.vision.sources.LimeLightSource;
import frc.robot.vision.sources.RobotPoseEstimatingVisionSource;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class MultiRobotVisionSources extends MultiVisionSources<RobotPoseEstimatingVisionSource> {

	public void switchLimelightsToOldBotPose(boolean useOldBotPose) {
		getVisionSources().forEach(visionSource -> {
			if (visionSource instanceof LimeLightSource) {
				((LimeLightSource) visionSource).switchToOldBotPose(useOldBotPose);
			}
		});
	}

	public void updateGyroAngles(GyroAngleValues gyroAngleValues) {
		getVisionSources().forEach(source -> source.updateGyroAngles(gyroAngleValues));
	}

	public List<Rotation2d> getAllRobotHeadingEstimations() {
		List<Rotation2d> headingEstimations = new ArrayList<>();
		getVisionSources().forEach(visionSource -> {
			Optional<Rotation2d> heading = visionSource.getRobotHeading();
			heading.ifPresent(headingEstimations::add);
		});
		return headingEstimations;
	}

}
