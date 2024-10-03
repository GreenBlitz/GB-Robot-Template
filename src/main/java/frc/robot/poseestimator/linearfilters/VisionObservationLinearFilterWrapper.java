package frc.robot.poseestimator.linearfilters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import frc.robot.poseestimator.observations.VisionObservation;

import java.util.*;

public class VisionObservationLinearFilterWrapper {

	private final ILinearFilter xFilter;
	private final ILinearFilter yFilter;
	private final ILinearFilter angleFilter;
	private final Stack<Pose2d> updatedObservations;

	/**
	 * A wrapper for {@code ILinearFilter} using odometry and vision
	 *
	 * @param logPath:    like, the log path
	 * @param filterType: the type of the linear filter that would be applied
	 * @param modifier:   modify the behavior of the filter. In case of FIR filters, this would be the sample count (casted to an
	 *                    integer), and for IIR filters the time constant (the period is always the RobotRIO cycle time).
	 */
	public VisionObservationLinearFilterWrapper(String logPath, LinearFilterType filterType, double modifier) {
		xFilter = LinearFilterFactory.create(filterType, logPath, modifier);
		yFilter = LinearFilterFactory.create(filterType, logPath, modifier);
		angleFilter = LinearFilterFactory.create(filterType, logPath, modifier);
		this.updatedObservations = new Stack<>();
	}

	/**
	 *
	 * adds new data to the filter, transformed by the new odometry data from the within a time period. That way, the old vision
	 * observation would be updated to its new approximated position
	 *
	 * @param visionObservation:             the observation that needs to be fixed and added
	 * @param newOdometryStartingTimestamps: the starting of the new odometry data
	 * @param newOdometrySEndingTimestamps:  the ending of the new odometry data
	 * @param odometryObservationsOverTime:  interpolated buffer contains the data from the starting to the ending of the new
	 *                                       odometry data
	 */
	public void addFixedData(
		Pose2d visionObservation,
		double newOdometryStartingTimestamps,
		double newOdometrySEndingTimestamps,
		TimeInterpolatableBuffer<Pose2d> odometryObservationsOverTime
	) {
		Optional<Pose2d> odometryStartingPose = odometryObservationsOverTime.getSample(newOdometryStartingTimestamps);
		Optional<Pose2d> odometryEndingPose = odometryObservationsOverTime.getSample(newOdometrySEndingTimestamps);

		if (odometryStartingPose.isPresent() && odometryEndingPose.isPresent()) {
			updatedObservations
				.add(fixVisionPose(visionObservation, odometryStartingPose.get(), odometryEndingPose.get()));
		}
	}

	public void addRawData(VisionObservation data) {
		updatedObservations.add(data.visionPose());
	}

	private Pose2d fixVisionPose(Pose2d visionPose, Pose2d odometryStartingPose, Pose2d odometryEndingPose) {
		Transform2d changeInPosition = odometryEndingPose.minus(odometryEndingPose);
		return visionPose.plus(changeInPosition);
	}

	public Pose2d calculateFilteredPose() {
		return new Pose2d(
			xFilter.calculateNewData(updatedObservations.getLast().getX()),
			yFilter.calculateNewData(updatedObservations.getLast().getY()),
			Rotation2d.fromRotations(angleFilter.calculateNewData(updatedObservations.getLast().getRotation().getRotations()))
		);
	}

}
