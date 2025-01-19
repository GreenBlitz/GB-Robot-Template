package frc.robot.vision.sources.limelights;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.signal.TimedValue;
import frc.robot.vision.GyroAngleValues;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.IndpendentHeadingVisionSource;
import frc.robot.vision.sources.RobotHeadingRequiringVisionSource;
import frc.utils.Filter;

import java.util.Optional;

/**
 * <code>DynamicSwitchingLimelight</code> is a class that automatically handles the switching between megatag 1 and megatag 2. It means it can
 * update the limelight, give the independent heading from the correct entry, switch between megatag 1 and 2 on runtime, and more.
 * <code>MultiAprilTagVisionSources</code> can make use of special methods of this class.
 */
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
		return useGyroForPoseEstimating ? headingRequiredLimelight.getVisionData() : independentPoseEstimatingLimelight.getVisionData();
	}

	@Override
	public Optional<AprilTagVisionData> getFilteredVisionData() {
		return useGyroForPoseEstimating
			? headingRequiredLimelight.getFilteredVisionData()
			: independentPoseEstimatingLimelight.getFilteredVisionData();
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
