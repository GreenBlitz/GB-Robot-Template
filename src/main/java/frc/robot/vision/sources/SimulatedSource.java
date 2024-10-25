package frc.robot.vision.sources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.GyroAngleValues;
import frc.robot.vision.RawVisionData;

import java.util.Optional;

public class SimulatedSource implements VisionSource<RawVisionData> {

	@Override
	public void updateGyroAngles(GyroAngleValues gyroAngleValues) {

	}

	@Override
	public void update() {

	}

	@Override
	public Optional<RawVisionData> getAllData() {
		return Optional.empty();
	}

	@Override
	public Optional<Rotation2d> getRobotHeading() {
		return Optional.empty();
	}

	@Override
	public void updateCurrentEstimatedPose(Pose2d estimatedPose) {

	}

}
