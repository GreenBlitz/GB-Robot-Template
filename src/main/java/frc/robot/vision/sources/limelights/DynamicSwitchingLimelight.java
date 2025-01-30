package frc.robot.vision.sources.limelights;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.VisionConstants;
import frc.utils.TimedValue;
import frc.robot.vision.RobotAngleValues;
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
	private final RobotHeadingRequiringVisionSource headingRequiringLimelight;
	private boolean useGyroForPoseEstimating;

	public DynamicSwitchingLimelight(
		boolean defaultUseGyroForPoseEstimating,
		String cameraNetworkTablesName,
		String parentLogPath,
		String sourceName,
		Filter<? super AprilTagVisionData> filter,
		Pose3d cameraPoseOffset
	) {
		this.useGyroForPoseEstimating = defaultUseGyroForPoseEstimating;
		this.independentPoseEstimatingLimelight = LimelightFactory.createRobotHeadingEstimatingLimelight(
			cameraNetworkTablesName,
			parentLogPath,
			sourceName + "/" + VisionConstants.DYNAMIC_LIMELIGHT_MEGATAG1_SOURCE_NAME,
			filter,
			cameraPoseOffset
		);
		this.headingRequiringLimelight = LimelightFactory.createRobotHeadingRequiringLimelight(
			cameraNetworkTablesName,
			parentLogPath,
			sourceName + "/" + VisionConstants.DYNAMIC_LIMELIGHT_MEGATAG2_SOURCE_NAME,
			filter,
			cameraPoseOffset
		);
	}

	public void setUseRobotHeadingForPoseEstimating(boolean useGyroForPoseEstimating) {
		this.useGyroForPoseEstimating = useGyroForPoseEstimating;
	}

	@Override
	public void update() {
		independentPoseEstimatingLimelight.update();
		headingRequiringLimelight.update();
	}

	@Override
	public Optional<AprilTagVisionData> getVisionData() {
		return useGyroForPoseEstimating ? headingRequiringLimelight.getVisionData() : independentPoseEstimatingLimelight.getVisionData();
	}

	@Override
	public Optional<AprilTagVisionData> getFilteredVisionData() {
		return useGyroForPoseEstimating
			? headingRequiringLimelight.getFilteredVisionData()
			: independentPoseEstimatingLimelight.getFilteredVisionData();
	}

	@Override
	public void setFilter(Filter<? super AprilTagVisionData> newFilter) {
		independentPoseEstimatingLimelight.setFilter(newFilter);
		headingRequiringLimelight.setFilter(newFilter);
	}

	@Override
	public Filter<? super AprilTagVisionData> getFilter() {
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
	public void updateRobotAngleValues(RobotAngleValues robotAngleValues) {
		headingRequiringLimelight.updateRobotAngleValues(robotAngleValues);
	}

}
