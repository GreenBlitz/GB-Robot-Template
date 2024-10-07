package frc.robot.poseestimator.linearfilters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.poseestimator.observations.VisionObservation;

import java.util.*;

public class VisionObservationLinearFilterWrapper {

	private final GBLinearFilter xFilter;
	private final GBLinearFilter yFilter;
	private final GBLinearFilter angleFilter;
	private final Stack<VisionObservation> internalData;
	private Pose2d LastObservation;

	/**
	 * A wrapper for {@code ILinearFilter} using odometry and vision
	 *
	 * @param logPath:    like, the log path
	 * @param filterType: the type of the linear filter that would be applied
	 * @param modifier:   modify the behavior of the filter. In case of FIR filters, this would be the sample count (casted to an integer), and
	 *                    for IIR filters the time constant (the period is always the RobotRIO cycle time).
	 */
	public VisionObservationLinearFilterWrapper(String logPath, LinearFilterType filterType, double modifier) {
		this.xFilter = LinearFilterFactory.create(filterType, logPath, modifier);
		this.yFilter = LinearFilterFactory.create(filterType, logPath, modifier);
		this.angleFilter = LinearFilterFactory.create(filterType, logPath, modifier);
		this.internalData = new Stack<>();
		this.LastObservation = new Pose2d();
	}

	/**
	 *
	 * adds new data to the filter, transformed by the new odometry data from the within a time period. That way, the old vision observation
	 * would be updated to its new approximated position
	 *
	 * @param visionObservation:             the observation that needs to be fixed and added
	 * @param newOdometryStartingTimestamps: the starting of the new odometry data
	 * @param newOdometrySEndingTimestamps:  the ending of the new odometry data
	 * @param odometryObservationsOverTime:  interpolated buffer contains the data from the starting to the ending of the new odometry data
	 */
	private Optional<Pose2d> getFixedData(
		Pose2d visionObservation,
		double newOdometryStartingTimestamps,
		double newOdometrySEndingTimestamps,
		TimeInterpolatableBuffer<Pose2d> odometryObservationsOverTime
	) {
		Optional<Pose2d> odometryStartingPose = odometryObservationsOverTime.getSample(newOdometryStartingTimestamps);
		Optional<Pose2d> odometryEndingPose = odometryObservationsOverTime.getSample(newOdometrySEndingTimestamps);

		if (odometryStartingPose.isPresent() && odometryEndingPose.isPresent()) {
			return Optional.of(fixVisionPose(visionObservation, odometryStartingPose.get(), odometryEndingPose.get()));
		}
		return Optional.empty();
	}

	public void addData(VisionObservation data) {
		LastObservation = data.robotPose();
		internalData.add(data);
		checkSize();
	}

	private Pose2d fixVisionPose(Pose2d visionPose, Pose2d odometryStartingPose, Pose2d odometryEndingPose) {
		Transform2d changeInPosition = odometryStartingPose.minus(odometryEndingPose);
		return visionPose.plus(changeInPosition);
	}

	public Pose2d calculateFilteredPose() {
		return new Pose2d(
			xFilter.calculateNewData(LastObservation.getX()),
			yFilter.calculateNewData(LastObservation.getY()),
			Rotation2d.fromRotations(angleFilter.calculateNewData(LastObservation.getRotation().getRotations()))
		);
	}

	public Pose2d calculateFixedData(TimeInterpolatableBuffer<Pose2d> odometryObservationsOverTime) {
		// ! bad code. pls dont' merge this, or fix this. Several side effects.
		double timestamp = Timer.getFPGATimestamp();
		xFilter.getFilter().reset();
		yFilter.getFilter().reset();
		angleFilter.getFilter().reset();
		for (VisionObservation data : internalData) {
			Optional<Pose2d> fixed = getFixedData(data.robotPose(), data.timestamp(), timestamp, odometryObservationsOverTime);
			fixed.ifPresent((Pose2d fixedData) -> {
				xFilter.calculateNewData(fixedData.getX());
				yFilter.calculateNewData(fixedData.getY());
				Rotation2d.fromRotations(angleFilter.calculateNewData(fixedData.getRotation().getRotations()));
			});
		}
		return new Pose2d(
			xFilter.getFilter().lastValue(),
			yFilter.getFilter().lastValue(),
			Rotation2d.fromRotations(angleFilter.calculateNewData(LastObservation.getRotation().getRotations()))
		);
	}

	protected void checkSize() {
		if (internalData.size() >= LinearFiltersConstants.MAX_SIZE) {
			internalData.removeLast();
		}
	}

}
