package frc.robot.vision.sources.limelights;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.signal.TimedValue;
import frc.robot.vision.GyroAngleValues;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.IndpendentHeadingVisionSource;
import frc.robot.vision.sources.RobotHeadingRequiringVisionSource;
import frc.robot.vision.sources.VisionSource;
import frc.utils.Filter;

import java.util.Optional;

public class DynamicSwitchingLimelight
	implements
		VisionSource<AprilTagVisionData>,
		IndpendentHeadingVisionSource,
		RobotHeadingRequiringVisionSource
{

	private final IndpendentHeadingVisionSource noisyLimelight;
	private final RobotHeadingRequiringVisionSource headingRequiredLimelight;
	private boolean useGyroForPoseEstimating;

	public DynamicSwitchingLimelight(
		boolean defaultUseGyroForPoseEstimating,
		String name,
		String parentLogPath,
		Filter<AprilTagVisionData> filter
	) {
		this.useGyroForPoseEstimating = defaultUseGyroForPoseEstimating;
		this.noisyLimelight = LimelightFactory.createRobotHeadingEstimatingLimelight(name, parentLogPath, filter);
		this.headingRequiredLimelight = LimelightFactory.createRobotHeadingRequiringLimelight(name, parentLogPath, filter);
	}

	public void useRobotHeadingForPoseEstimating(boolean useGyroForPoseEstimating) {
		this.useGyroForPoseEstimating = useGyroForPoseEstimating;
	}

	@Override
	public void update() {
		noisyLimelight.update();
		headingRequiredLimelight.update();
	}

	@Override
	public Optional<AprilTagVisionData> getVisionData() {
		return useGyroForPoseEstimating ? noisyLimelight.getVisionData() : headingRequiredLimelight.getVisionData();
	}

	@Override
	public Optional<AprilTagVisionData> getFilteredVisionData() {
		return useGyroForPoseEstimating ? noisyLimelight.getFilteredVisionData() : headingRequiredLimelight.getFilteredVisionData();
	}

	@Override
	public void setFilter(Filter<AprilTagVisionData> newFilter) {
		noisyLimelight.setFilter(newFilter);
		headingRequiredLimelight.setFilter(newFilter);
	}

	@Override
	public Filter<AprilTagVisionData> getFilter() {
		return noisyLimelight.getFilter(); // same filter for both sources
	}

	@Override
	public Optional<TimedValue<Rotation2d>> getRawHeadingData() {
		return noisyLimelight.getRawHeadingData();
	}

	@Override
	public Optional<TimedValue<Rotation2d>> getFilteredHeadingData() {
		return noisyLimelight.getFilteredHeadingData();
	}

	@Override
	public void updateGyroAngleValues(GyroAngleValues gyroAngleValues) {
		headingRequiredLimelight.updateGyroAngleValues(gyroAngleValues);
	}

}
