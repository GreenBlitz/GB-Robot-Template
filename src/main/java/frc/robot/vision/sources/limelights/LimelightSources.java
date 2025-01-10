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
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class LimelightSources {

	public static class HeadingRequiredLimelight extends LimeLightSource implements RobotHeadingRequiringVisionSource {

		private GyroAngleValues gyroAngleValues;
		private final NetworkTableEntry robotOrientationEntry;

		public HeadingRequiredLimelight(String name, String parentLogPath, Filter<AprilTagVisionData> filter) {
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

	}

	public static class DynamicLimelight
		implements
			VisionSource<AprilTagVisionData>,
			IndpendentHeadingVisionSource,
			RobotHeadingRequiringVisionSource
	{

		private final NoisyLimelight noisyLimelight;
		private final HeadingRequiredLimelight headingRequiredLimelight;
		private boolean useGyroForPoseEstimating;

		public DynamicLimelight(boolean defaultUseGyroForPoseEstimating, String name, String parentLogPath, Filter<AprilTagVisionData> filter) {
			this.useGyroForPoseEstimating = defaultUseGyroForPoseEstimating;
			this.noisyLimelight = new NoisyLimelight(name, parentLogPath, filter);
			this.headingRequiredLimelight = new HeadingRequiredLimelight(name, parentLogPath, filter);
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
			return noisyLimelight.filter; // same filter for both sources
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
			headingRequiredLimelight.updateGyroAngleValues(gyroAngleValues);
		}

	}

}
