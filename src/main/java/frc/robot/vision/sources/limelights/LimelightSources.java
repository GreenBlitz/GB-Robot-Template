package frc.robot.vision.sources.limelights;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.vision.GyroAngleValues;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.data.HeadingData;
import frc.robot.vision.sources.IndpendentHeadingVisionSource;
import frc.robot.vision.sources.RobotHeadingRequiringVisionSource;
import frc.robot.vision.sources.VisionSource;
import frc.utils.Filter;
import frc.utils.pose.PoseUtils;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.BiFunction;

public class LimelightSources {

	public static class GyroSupportingLimelight extends LimeLightSource implements RobotHeadingRequiringVisionSource {

		private GyroAngleValues gyroAngleValues;
		private final NetworkTableEntry robotOrientationEntry;

		public GyroSupportingLimelight(String name, String parentLogPath, Filter<AprilTagVisionData> filter) {
			super(name, parentLogPath, filter, LimelightPoseEstimatingMethod.BOTPOSE_2);
			this.gyroAngleValues = new GyroAngleValues(Rotation2d.fromDegrees(0), 0, Rotation2d.fromDegrees(0), 0, Rotation2d.fromDegrees(0), 0);
			this.robotOrientationEntry = super.getLimelightNetworkTableEntry("robot_orientation_set");
		}

		public void updateGyroAngleValues(GyroAngleValues gyroAngleValues) {
			this.gyroAngleValues = gyroAngleValues;
		}

		@Override
		public void update() {
			super.update();
			robotOrientationEntry.setDoubleArray(gyroAngleValues.asArray());
			Logger.recordOutput(logPath + "gyroAngleValues", gyroAngleValues.asArray());
		}

	}

	public static class NoisyLimelight extends LimeLightSource implements IndpendentHeadingVisionSource {

		public NoisyLimelight(String name, String parentLogPath, Filter<AprilTagVisionData> filter) {
			super(name, parentLogPath, filter, LimelightPoseEstimatingMethod.BOTPOSE_1);
		}

		@Override
		public void log() {
			super.log();
			getVisionData().ifPresent(
				(heading) -> Logger.recordOutput(super.logPath + "robotBotPose1Heading", heading.getEstimatedPose().getRotation().toRotation2d())
			);
		}

		@Override
		public Optional<HeadingData> getRawHeadingData() {
			return getVisionData().map(PoseUtils::VisionDataToHeadingData);
		}

		@Override
		public Optional<HeadingData> getFilteredHeadingData() {
			return getFilteredVisionData().map(PoseUtils::VisionDataToHeadingData);
		}

	}

	public static class DynamicLimelight
		implements
			VisionSource<AprilTagVisionData>,
			IndpendentHeadingVisionSource,
			RobotHeadingRequiringVisionSource
	{

		private final NoisyLimelight noisyLimelight;
		private final GyroSupportingLimelight gyroSupportingLimelight;
		private boolean useGyroForPoseEstimating;

		public DynamicLimelight(boolean defaultUseGyroForPoseEstimating, String name, String parentLogPath, Filter<AprilTagVisionData> filter) {
			this.useGyroForPoseEstimating = defaultUseGyroForPoseEstimating;
			this.noisyLimelight = new NoisyLimelight(name, parentLogPath, filter);
			this.gyroSupportingLimelight = new GyroSupportingLimelight(name, parentLogPath, filter);
		}

		public void useRobotHeadingForPoseEstimating(boolean useGyroForPoseEstimating) {
			this.useGyroForPoseEstimating = useGyroForPoseEstimating;
		}

		@Override
		public void update() {
			noisyLimelight.update();
			gyroSupportingLimelight.update();
		}

		@Override
		public Optional<AprilTagVisionData> getVisionData() {
			return useGyroForPoseEstimating ? noisyLimelight.getVisionData() : gyroSupportingLimelight.getVisionData();
		}

		@Override
		public Optional<AprilTagVisionData> getFilteredVisionData() {
			return useGyroForPoseEstimating ? noisyLimelight.getFilteredVisionData() : gyroSupportingLimelight.getFilteredVisionData();
		}

		@Override
		public Filter<AprilTagVisionData> setFilter(Filter<AprilTagVisionData> newFilter) {
			noisyLimelight.setFilter(newFilter);
			return gyroSupportingLimelight.setFilter(newFilter);
		}

		@Override
		public Filter<AprilTagVisionData> applyOnFilter(
			BiFunction<Filter<AprilTagVisionData>, Filter<AprilTagVisionData>, Filter<AprilTagVisionData>> applicationFunction,
			Filter<AprilTagVisionData> filterToApplyWith
		) {
			noisyLimelight.applyOnFilter(applicationFunction, filterToApplyWith);
			return gyroSupportingLimelight.applyOnFilter(applicationFunction, filterToApplyWith);
		}

		@Override
		public Optional<HeadingData> getRawHeadingData() {
			return noisyLimelight.getRawHeadingData();
		}

		@Override
		public Optional<HeadingData> getFilteredHeadingData() {
			return noisyLimelight.getFilteredHeadingData();
		}

		@Override
		public void updateGyroAngleValues(GyroAngleValues gyroAngleValues) {
			gyroSupportingLimelight.updateGyroAngleValues(gyroAngleValues);
		}

	}

}
