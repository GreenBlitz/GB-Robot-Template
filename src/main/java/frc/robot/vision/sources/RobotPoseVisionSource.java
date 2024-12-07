package frc.robot.vision.sources;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.rawdata.RawVisionAprilTagData;

import java.util.Optional;

public interface RobotPoseVisionSource extends VisionSource<RawVisionAprilTagData> {

	Optional<Rotation2d> getRobotHeading();

}
