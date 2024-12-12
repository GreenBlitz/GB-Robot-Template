package frc.robot.vision.multivisionsources;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.rawdata.RawAprilTagVisionData;
import frc.robot.vision.rawdata.RawVisionData;
import frc.robot.vision.sources.LimeLightSource;
import frc.robot.vision.sources.LimelightGyroAngleValues;
import frc.robot.vision.sources.VisionSource;

import java.util.ArrayList;
import java.util.List;

public class MultiVisionSourcesWithExtendedLimelightSupport extends MultiVisionSources<VisionSource<? extends RawAprilTagVisionData>> {

	@SafeVarargs
	public MultiVisionSourcesWithExtendedLimelightSupport(String logPath, VisionSource<? extends RawAprilTagVisionData>... visionSources) {
		super(logPath, visionSources);
	}

	public MultiVisionSourcesWithExtendedLimelightSupport(String logPath, List<VisionSource<? extends RawAprilTagVisionData>> visionSources) {
		super(logPath, visionSources);
	}

	public void updateYawInLimelights(Rotation2d yaw) {
		for (VisionSource<? extends RawAprilTagVisionData> visionSource : getVisionSources()) {
			if (visionSource instanceof LimeLightSource limelightSource) {
				limelightSource
					.updateGyroAngles(new LimelightGyroAngleValues(yaw, 0, Rotation2d.fromDegrees(0), 0, Rotation2d.fromDegrees(0), 0));
			}
		}
	}

	public ArrayList<Rotation2d> getRawEstimatedAngles() {
		ArrayList<Rotation2d> output = new ArrayList<>();
		for (VisionSource<? extends RawAprilTagVisionData> visionSource : getVisionSources()) {
			if (visionSource instanceof LimeLightSource limeLightSource) {
				limeLightSource.getRobotHeading().ifPresent(output::add);
			} else {
				visionSource.getRawVisionData()
					.ifPresent(
						(RawVisionData visionData) -> output.add(Rotation2d.fromRadians(visionData.getEstimatedPose().getRotation().getZ()))
					);
			}
		}
		return output;
	}

}
