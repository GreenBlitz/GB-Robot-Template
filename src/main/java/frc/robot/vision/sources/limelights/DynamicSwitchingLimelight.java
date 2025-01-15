package frc.robot.vision.sources.limelights;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.signal.TimedValue;
import frc.robot.vision.GyroAngleValues;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.IndpendentHeadingVisionSource;
import frc.robot.vision.sources.RobotHeadingRequiringVisionSource;
import frc.utils.Filter;

import java.util.Optional;

public class DynamicSwitchingLimelight implements IndpendentHeadingVisionSource, RobotHeadingRequiringVisionSource {

	private final IndpendentHeadingVisionSource independentPoseEstimatingLimelight;
	private final RobotHeadingRequiringVisionSource headingRequiredLimelight;
	private boolean useGyroForPoseEstimating;

	public DynamicSwitchingLimelight(
		boolean defaultUseGyroForPoseEstimating,
		String name,
		String parentLogPath,
		Filter<AprilTagVisionData> filter
	) {
		this.useGyroForPoseEstimating = defaultUseGyroForPoseEstimating;
		this.independentPoseEstimatingLimelight = LimelightFactory.createRobotHeadingEstimatingLimelight(name, parentLogPath, filter);
		this.headingRequiredLimelight = LimelightFactory.createRobotHeadingRequiringLimelight(name, parentLogPath, filter);
	}

	public void setUseRobotHeadingForPoseEstimating(boolean useGyroForPoseEstimating) {
		this.useGyroForPoseEstimating = useGyroForPoseEstimating;
	}

	@Override
	public void update() {
		independentPoseEstimatingLimelight.update();
		headingRequiredLimelight.update();
	}

	@Override
	public Optional<AprilTagVisionData> getVisionData() {
		return useGyroForPoseEstimating ? independentPoseEstimatingLimelight.getVisionData() : headingRequiredLimelight.getVisionData();
	}

	@Override
	public Optional<AprilTagVisionData> getFilteredVisionData() {
		return useGyroForPoseEstimating
			? independentPoseEstimatingLimelight.getFilteredVisionData()
			: headingRequiredLimelight.getFilteredVisionData();
	}

	@Override
	public void setFilter(Filter<AprilTagVisionData> newFilter) {
		independentPoseEstimatingLimelight.setFilter(newFilter);
		headingRequiredLimelight.setFilter(newFilter);
	}

	@Override
	public Filter<AprilTagVisionData> getFilter() {
		return independentPoseEstimatingLimelight.getFilter(); // same filter for both sources
	}

	@Override
	public Optional<TimedValue<Rotation2d>> getRawHeadingData() {
		return independentPoseEstimatingLimelight.getRawHeadingData();
	}

	@Override
	public Optional<TimedValue<Rotation2d>> getFilteredHeadingData() {
		return independentPoseEstimatingLimelight.getFilteredHeadingData();
	}

	@Override
	public void updateGyroAngleValues(GyroAngleValues gyroAngleValues) {
		headingRequiredLimelight.updateGyroAngleValues(gyroAngleValues);
	}

}
