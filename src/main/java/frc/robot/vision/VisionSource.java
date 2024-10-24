package frc.robot.vision;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;

public interface VisionSource<VisionData> {

	void updateGyroAngles(GyroAngleValues gyroAngleValues);

	void update();

	Optional<VisionData> getAllData();

	Optional<Rotation2d> getRobotHeading();

}
