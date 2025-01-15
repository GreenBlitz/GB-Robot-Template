package frc.robot.poseestimator;

import edu.wpi.first.math.geometry.*;
import frc.robot.vision.data.VisionData;

public class PoseEstimationMath {

	public static double deriveVisionData(VisionData starting, VisionData finish) {
		double deltaTime = finish.getTimestamp() - starting.getTimestamp();
		Pose2d startingPose = starting.getEstimatedPose().toPose2d();
		Pose2d finishingPose = finish.getEstimatedPose().toPose2d();
		return startingPose.minus(finishingPose).getTranslation().getNorm() / deltaTime;
	}

	public static double deriveTwist(Twist2d twist, double deltaTime) {
		double distance = Math.sqrt((Math.pow(twist.dx, 2) + Math.pow(twist.dy, 2)));
		return distance / deltaTime;
	}

}
